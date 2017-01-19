
AM_CXXFLAGS = \
	$(GLIB_CFLAGS) \
	$(EXAMPLES_FREENECT2_DEPS_CFLAGS) \
	-I$(top_srcdir)/@PRJ_NAME@

test_kinect2_LDADD = \
	$(top_builddir)/@PRJ_NAME@/lib@PRJ_API_NAME@.la \
	$(EXAMPLES_FREENECT2_DEPS_LIBS)

noinst_PROGRAMS = \
	test-kinect2

test_kinect2_SOURCES = test-kinect2.cc

MAINTAINERCLEANFILES = Makefile.in *~

DISTCLEANFILES = $(MAINTAINERCLEANFILES)
