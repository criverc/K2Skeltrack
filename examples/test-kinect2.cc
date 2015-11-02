#include <iostream>
#include <string.h>
#include <signal.h>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>

#include <skeltrack.h>
#include <math.h>
#include <cairo.h>
#include <glib-object.h>
#include <clutter/clutter.h>
#include <clutter/clutter-keysyms.h>

static SkeltrackSkeleton *skeleton = NULL;
static ClutterActor *info_text;
static ClutterActor *skel_tex;
static ClutterActor *depth_tex;
static ClutterActor *video_tex;
static ClutterContent *depth_canvas;
static ClutterContent *depth_image = NULL;
static SkeltrackJointList list = NULL;

static gboolean SHOW_SKELETON = TRUE;
static gboolean ENABLE_SMOOTHING = TRUE;
static gfloat SMOOTHING_FACTOR = .5;

static guint THRESHOLD_BEGIN = 500;
/* Adjust this value to increase of decrease
   the threshold */
static guint THRESHOLD_END   = 3000;
static gint DIMENSION_REDUCTION = 8;

void* kinect = NULL;

libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
libfreenect2::FrameMap frames;
libfreenect2::Registration* registration = NULL;
libfreenect2::Frame undistorted(512, 424, 4);
libfreenect2::Frame registered(512, 424, 4);
size_t framecount = 0;


typedef struct
{
  guint16 *reduced_buffer;
  gint width;
  gint height;
  gint reduced_width;
  gint reduced_height;
} BufferInfo;

static void
on_track_joints (GObject      *obj,
                 GAsyncResult *res,
                 gpointer      user_data)
{
  guint i;
  BufferInfo *buffer_info;
  guint16 *reduced;
  gint width, height, reduced_width, reduced_height;
  ClutterContent *content;
  GError *error = NULL;

  buffer_info = (BufferInfo *) user_data;
  reduced = (guint16 *) buffer_info->reduced_buffer;
  width = buffer_info->width;
  height = buffer_info->height;
  reduced_width = buffer_info->reduced_width;
  reduced_height = buffer_info->reduced_height;

  list = skeltrack_skeleton_track_joints_finish (skeleton,
                                                 res,
                                                 &error);

  if (error == NULL)
    {
      content = clutter_actor_get_content (skel_tex);
      clutter_content_invalidate (content);
    }
  else
    {
      g_warning ("%s\n", error->message);
      g_error_free (error);
    }

  g_slice_free1 (reduced_width * reduced_height * sizeof (guint16), reduced);

  g_slice_free (BufferInfo, buffer_info);

  skeltrack_joint_list_free (list);
}

static void
grayscale_buffer_set_value (guchar *buffer, gint index, guchar value)
{
  buffer[index * 3] = value;
  buffer[index * 3 + 1] = value;
  buffer[index * 3 + 2] = value;
}

static BufferInfo *
process_buffer (guint16 *buffer,
                guint width,
                guint height,
                guint dimension_factor,
                guint threshold_begin,
                guint threshold_end)
{
  BufferInfo *buffer_info;
  gint i, j, reduced_width, reduced_height;
  guint16 *reduced_buffer;

  g_return_val_if_fail (buffer != NULL, NULL);

  reduced_width = (width - width % dimension_factor) / dimension_factor;
  reduced_height = (height - height % dimension_factor) / dimension_factor;

  reduced_buffer = (guint16*) g_slice_alloc0 (reduced_width * reduced_height * sizeof (guint16));

  for (i = 0; i < reduced_width; i++)
    {
      for (j = 0; j < reduced_height; j++)
        {
          gint index;
          guint16 value;

          index = j * width * dimension_factor + i * dimension_factor;
          value = buffer[index];

          if (value < threshold_begin || value > threshold_end)
            {
              reduced_buffer[j * reduced_width + i] = 0;
              continue;
            }

          reduced_buffer[j * reduced_width + i] = value;
        }
    }

  buffer_info = g_slice_new0 (BufferInfo);
  buffer_info->reduced_buffer = reduced_buffer;
  buffer_info->reduced_width = reduced_width;
  buffer_info->reduced_height = reduced_height;
  buffer_info->width = width;
  buffer_info->height = height;

  return buffer_info;
}

static guchar *
create_grayscale_buffer (BufferInfo *buffer_info, gint dimension_reduction)
{
  gint i,j;
  gint size;
  guchar *grayscale_buffer;
  guint16 *reduced_buffer;

  reduced_buffer = buffer_info->reduced_buffer;

  size = buffer_info->width * buffer_info->height * sizeof (guchar) * 3;
  grayscale_buffer = (guchar*) g_slice_alloc (size);
  /*Paint is white*/
  memset (grayscale_buffer, 255, size);

  for (i = 0; i < buffer_info->reduced_width; i++)
    {
      for (j = 0; j < buffer_info->reduced_height; j++)
        {
 
          if (reduced_buffer[j * buffer_info->reduced_width + i] != 0)
            {
//              grayscale_buffer_set_value (grayscale_buffer, index, 0);
              gint index = j * dimension_reduction * buffer_info->width +
                i * dimension_reduction;

              grayscale_buffer[index * 3] = 255;
              grayscale_buffer[index * 3 + 1] = 0;
              grayscale_buffer[index * 3 + 2] = 0;

            }
         }
    }

  return grayscale_buffer;
}

static void
on_depth_frame (libfreenect2::Frame* frame)
{
  gboolean smoothing_enabled;
  gint width, height;
  gint dimension_factor = DIMENSION_REDUCTION;
  guchar *grayscale_buffer;
  guint16 *depth;
  BufferInfo *buffer_info;
  gsize len;
  GError *error = NULL;
  ClutterContent *content;

  width = frame->width;
  height = frame->height;

  depth = (guint16*) g_slice_alloc (width * height * sizeof (guint16));

  /* convert buffer from float* to guint16* so it can be processed by skeltrack */
  for (int i=0; i < height; i++) 
  {
    for (int j=0; j < width; j++)
    {
        depth[i*width + j] = (guint16) (((float*) (frame->data))[i*width+j]);
    }
  } 

  g_object_set (skeleton, "dimension-reduction", dimension_factor, NULL);
  
  buffer_info = process_buffer (depth,
                                width,
                                height,
                                dimension_factor,
                                THRESHOLD_BEGIN,
                                THRESHOLD_END);

  skeltrack_skeleton_track_joints (skeleton,
                                   buffer_info->reduced_buffer,
                                   buffer_info->reduced_width,
                                   buffer_info->reduced_height,
                                   NULL,
                                   on_track_joints,
                                   buffer_info);


  content = clutter_actor_get_content (depth_tex);
  grayscale_buffer = create_grayscale_buffer (buffer_info, dimension_factor);

  if (depth_image == NULL)
    depth_image = clutter_image_new ();

  clutter_actor_set_content (depth_tex, depth_image);
  if (! clutter_image_set_data (CLUTTER_IMAGE (depth_image),
                                grayscale_buffer,
                                COGL_PIXEL_FORMAT_RGB_888,
                                width,
                                height,
                                0,
                                &error))
    {
      g_debug ("Error setting texture area: %s", error->message);
      g_error_free (error);
    }

  g_slice_free1 (width * height * sizeof (guchar) * 3, grayscale_buffer);
  g_slice_free1 (width * height * sizeof (guint16), depth);
}

static void
on_video_frame (libfreenect2::Frame* frame)
{
  GError *error = NULL;
  ClutterContent *content;

  content = clutter_actor_get_content (video_tex);

  if (! clutter_image_set_data (CLUTTER_IMAGE (content),
                                frame->data,
                                COGL_PIXEL_FORMAT_BGRA_8888,
                                frame->width, frame->height,
                                0,
                                &error))
    {
      g_debug ("Error setting texture area: %s", error->message);
      g_error_free (error);
    }

}

static void
paint_joint (cairo_t *cairo,
             SkeltrackJoint *joint,
             gint radius,
             const gchar *color_str)
{
  ClutterColor *color;

  if (joint == NULL)
    return;

  color = clutter_color_new (0, 0, 0, 200);
  clutter_color_from_string (color, color_str);

  cairo_set_line_width (cairo, 10);
  clutter_cairo_set_source_color (cairo, color);
  cairo_arc (cairo,
             joint->screen_x,
             joint->screen_y,
             radius / joint->z,
             0,
             G_PI * 2);
  cairo_fill (cairo);
  clutter_color_free (color);
}

static void
connect_joints (cairo_t *cairo,
                SkeltrackJoint *joint_a,
                SkeltrackJoint *joint_b,
                const gchar *color_str)
{
  ClutterColor *color;

  if (joint_a == NULL || joint_b == NULL)
    return;

  color = clutter_color_new (0, 0, 0, 200);
  clutter_color_from_string (color, color_str);

  cairo_set_line_width (cairo, 10);
  clutter_cairo_set_source_color (cairo, color);
  cairo_move_to (cairo,
                 joint_a->screen_x,
                 joint_a->screen_y);
  cairo_line_to (cairo,
                 joint_b->screen_x,
                 joint_b->screen_y);
  cairo_stroke (cairo);
  clutter_color_free (color);
}

static gboolean
on_skeleton_draw (ClutterCanvas *canvas,
                  cairo_t *cairo,
                  gint width,
                  gint height,
                  gpointer user_data)
{
  ClutterColor *color;
  SkeltrackJoint *head, *left_hand, *right_hand,
    *left_shoulder, *right_shoulder, *left_elbow, *right_elbow;

  if (list == NULL)
    return FALSE;

  head = skeltrack_joint_list_get_joint (list,
                                         SKELTRACK_JOINT_ID_HEAD);
  left_hand = skeltrack_joint_list_get_joint (list,
                                              SKELTRACK_JOINT_ID_LEFT_HAND);
  right_hand = skeltrack_joint_list_get_joint (list,
                                               SKELTRACK_JOINT_ID_RIGHT_HAND);
  left_shoulder = skeltrack_joint_list_get_joint (list,
                                       SKELTRACK_JOINT_ID_LEFT_SHOULDER);
  right_shoulder = skeltrack_joint_list_get_joint (list,
                                       SKELTRACK_JOINT_ID_RIGHT_SHOULDER);
  left_elbow = skeltrack_joint_list_get_joint (list,
                                               SKELTRACK_JOINT_ID_LEFT_ELBOW);
  right_elbow = skeltrack_joint_list_get_joint (list,
                                                SKELTRACK_JOINT_ID_RIGHT_ELBOW);

  /* Paint it white */
  color = clutter_color_new (255, 255, 255, 255);
  clutter_cairo_set_source_color (cairo, color);
  cairo_rectangle (cairo, 0, 0, width, height);
  cairo_fill (cairo);
  clutter_color_free (color);

  paint_joint (cairo, head, 50000, "#FFF800");

  connect_joints (cairo, left_shoulder, right_shoulder, "#afafaf");

  connect_joints (cairo, left_shoulder, left_hand, "#afafaf");

  connect_joints (cairo, right_shoulder, right_hand, "#afafaf");

//  connect_joints (cairo, right_hand, right_elbow, "#afafaf");

//  connect_joints (cairo, left_hand, left_elbow, "#afafaf");

  paint_joint (cairo, left_hand, 30000, "#C2FF00");

  paint_joint (cairo, left_shoulder, 20000, "#C2FF00");

//  paint_joint (cairo, left_elbow, 20, "#C2FF00");

  paint_joint (cairo, right_hand, 30000, "#00FAFF");

  paint_joint (cairo, right_shoulder, 20000, "#00FAFF");

//  paint_joint (cairo, right_elbow, 20, "#00FAFF");

  skeltrack_joint_list_free (list);
  list = NULL;

  return FALSE;
}

static void
set_info_text (void)
{
  gchar *title;
  title = g_strdup_printf ("<b>Current View:</b> %s\t\t\t"
                           "<b>Threshold:</b> %d\n"
                           "<b>Smoothing Enabled:</b> %s\t\t\t"
                           "<b>Smoothing Level:</b> %.2f",
                           SHOW_SKELETON ? "Skeleton" : "Point Cloud",
                           THRESHOLD_END,
                           ENABLE_SMOOTHING ? "Yes" : "No",
                           SMOOTHING_FACTOR);
  clutter_text_set_markup (CLUTTER_TEXT (info_text), title);
  g_free (title);
}

static void
set_threshold (gint difference)
{
  gint new_threshold = THRESHOLD_END + difference;
  if (new_threshold >= THRESHOLD_BEGIN + 300 &&
      new_threshold <= 4000)
    THRESHOLD_END = new_threshold;
}

static void
set_tilt_angle (void *kinect, gdouble difference)
{
    printf ("nothing to do here, kinect v2 has no motor ...\n");
    fflush (0);
}

static void
enable_smoothing (gboolean enable)
{
  if (skeleton != NULL)
    g_object_set (skeleton, "enable-smoothing", enable, NULL);
}

static void
set_smoothing_factor (gfloat factor)
{
  if (skeleton != NULL)
    {
      SMOOTHING_FACTOR += factor;
      SMOOTHING_FACTOR = CLAMP (SMOOTHING_FACTOR, 0.0, 1.0);
      g_object_set (skeleton, "smoothing-factor", SMOOTHING_FACTOR, NULL);
    }
}

static gboolean
on_key_release (ClutterActor *actor,
                ClutterEvent *event,
                gpointer data)
{
  void *kinect;
  gdouble angle;
  guint key;
  g_return_val_if_fail (event != NULL, FALSE);

  key = clutter_event_get_key_symbol (event);
  switch (key)
    {
    case CLUTTER_KEY_space:
      SHOW_SKELETON = !SHOW_SKELETON;
      if (SHOW_SKELETON) clutter_actor_set_opacity (skel_tex, 122);
      else               clutter_actor_set_opacity (skel_tex, 0);
      break;
    case CLUTTER_KEY_plus:
      set_threshold (10);
      break;
    case CLUTTER_KEY_minus:
      set_threshold (-10);
      break;
    case CLUTTER_KEY_Up:
      set_tilt_angle (kinect, 5);
      break;
    case CLUTTER_KEY_Down:
      set_tilt_angle (kinect, -5);
      break;
    case CLUTTER_KEY_s:
      ENABLE_SMOOTHING = !ENABLE_SMOOTHING;
      enable_smoothing (ENABLE_SMOOTHING);
      break;
    case CLUTTER_KEY_Right:
      set_smoothing_factor (.05);
      break;
    case CLUTTER_KEY_Left:
      set_smoothing_factor (-.05);
      break;
    }
  set_info_text ();
  return TRUE;
}

static ClutterActor *
create_instructions (void)
{
  ClutterActor *text;

  text = clutter_text_new ();
  clutter_text_set_markup (CLUTTER_TEXT (text),
                         "<b>Instructions:</b>\n"
                         "\tChange between skeleton\n"
                         "\t  tracking and threshold view:  \tSpace bar\n"
                         "\tSet tilt angle:  \t\t\t\tUp/Down Arrows\n"
                         "\tIncrease threshold:  \t\t\t+/-\n"
                         "\tEnable/Disable smoothing:  \t\ts\n"
                         "\tSet smoothing level:  \t\t\tLeft/Right Arrows\n"
                           );
  return text;
}

static void
on_destroy (ClutterActor *actor, gpointer data)
{
  clutter_main_quit ();
}

static void
quit (gint signale)
{
  signal (SIGINT, 0);

  clutter_main_quit ();
}


void on_timeline_new_frame (ClutterTimeline *timeline, gint frame_num, gpointer data) 
{
    listener.waitForNewFrame (frames);
    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

    registration->apply (rgb, depth, &undistorted, &registered);

    framecount++;
    if (framecount % 100 == 0)
        std::cout << "Received " << framecount << " frames." << std::endl;

    on_video_frame (rgb);
    on_depth_frame (depth);

    listener.release (frames);
}

int
main (int argc, char *argv[])
{
  ClutterActor *stage, *instructions;
  GError *error = NULL;
  gint width = 512;
  gint height = 424;

  if (clutter_init (&argc, &argv) != CLUTTER_INIT_SUCCESS)
    return -1;

  stage = clutter_stage_new ();
  clutter_stage_set_title (CLUTTER_STAGE (stage), "Kinect v2 Test");
  clutter_actor_set_size (stage, width * 2, height + 250);
  clutter_stage_set_user_resizable (CLUTTER_STAGE (stage), TRUE);

  g_signal_connect (stage, "destroy", G_CALLBACK (on_destroy), NULL);
  g_signal_connect (stage, "key-release-event", G_CALLBACK (on_key_release), NULL);

  depth_tex = clutter_actor_new ();
  clutter_actor_set_size (depth_tex, width, height);
  clutter_actor_add_child (stage, depth_tex); 
  clutter_actor_set_position (depth_tex, 0.0, 0.0);

  skel_tex = clutter_actor_new ();
  depth_canvas = clutter_canvas_new ();
  clutter_actor_set_content (skel_tex, depth_canvas);
  clutter_canvas_set_size (CLUTTER_CANVAS (depth_canvas), width, height);
  clutter_actor_set_size (skel_tex, width, height);
  clutter_actor_add_child (stage, skel_tex);
  clutter_actor_set_position (skel_tex, 0.0, 0.0);
  clutter_actor_set_opacity (skel_tex, 122);

  video_tex = clutter_actor_new ();
  clutter_actor_set_content (video_tex, clutter_image_new ());
  clutter_actor_set_size (video_tex, width, height);
  clutter_actor_set_position (video_tex, width, 0.0);
  clutter_actor_add_child (stage, video_tex);

  info_text = clutter_text_new ();
  clutter_actor_set_position (info_text, 50, height + 20);
  clutter_actor_add_child (stage, info_text); 

  instructions = create_instructions ();
  clutter_actor_set_position (instructions, 50, height + 70);
  clutter_actor_add_child (stage, instructions);

  skeleton = skeltrack_skeleton_new ();
  g_object_get (skeleton, "smoothing-factor", &SMOOTHING_FACTOR, NULL);

  set_info_text ();

  g_signal_connect (depth_canvas,
                    "draw",
                    G_CALLBACK (on_skeleton_draw),
                    NULL);

  clutter_actor_show (stage);

  signal (SIGINT, quit);

  ClutterTimeline *timeline = clutter_timeline_new (1);
  g_signal_connect (timeline, "new-frame", G_CALLBACK (on_timeline_new_frame), NULL);
  clutter_timeline_set_repeat_count (timeline, -1);
  clutter_timeline_start (timeline);

  libfreenect2::Freenect2Device *dev = 0;
  libfreenect2::PacketPipeline *pipeline = 0;

  libfreenect2::Freenect2 freenect2;

  if(freenect2.enumerateDevices() == 0)
  {
    std::cout << "no device connected!" << std::endl;
    return -1;
  }

  std::string serial = freenect2.getDefaultDeviceSerialNumber();

  for(int argI = 1; argI < argc; ++argI)
  {
    const std::string arg(argv[argI]);

    if(arg == "cpu")
    {
      if(!pipeline)
        pipeline = new libfreenect2::CpuPacketPipeline();
    }
    else if(arg == "gl")
    {
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
      if(!pipeline)
        pipeline = new libfreenect2::OpenGLPacketPipeline();
#else
      std::cout << "OpenGL pipeline is not supported!" << std::endl;
#endif
    }
    else if(arg == "cl")
    {
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
      if(!pipeline)
        pipeline = new libfreenect2::OpenCLPacketPipeline();
#else
      std::cout << "OpenCL pipeline is not supported!" << std::endl;
#endif
    }
    else if(arg.find_first_not_of("0123456789") == std::string::npos) //check if parameter could be a serial number
    {
      serial = arg;
    }
    else
    {
      std::cout << "Unknown argument: " << arg << std::endl;
    }
  }

  if(pipeline)
  {
    dev = freenect2.openDevice(serial, pipeline);
  }
  else
  {
    dev = freenect2.openDevice(serial);
  }

  if(dev == 0)
  {
    std::cout << "failure opening device!" << std::endl;
    return -1;
  }

  dev->setColorFrameListener(&listener);
  dev->setIrAndDepthFrameListener(&listener);
  dev->start();

  std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
  std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

  registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());

  framecount = 0;

  clutter_main ();

  dev->stop();
  dev->close();

  delete registration;

  g_object_unref (timeline);

  if (kinect != NULL)
    g_object_unref (kinect);

  if (skeleton != NULL)
    {
      g_object_unref (skeleton);
    }

  return 0;
}

