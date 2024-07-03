SUBDIRS = attestlog_AD

EXTRA_DIST = m4/.keepdir
ACLOCAL_AMFLAGS = -I m4 --install

clean-local:
	rm -rf \
		autoscan.log \
		aclocal.m4 \
		autom4te.cache \
		build-aux \
		config.h \
		config.h.in \
		config.h.in~ \
		config.log \
		config.status \
		configure \
		libtool \
		m4/* \
		.deps \
		Makefile.in \
		Makefile \
		stamp-h1 \
		attestlog_AD/*.in \
		attestlog_AD/*.o \
		attestlog_AD/*.la \
		attestlog_AD/*.lo \
		attestlog_AD/main \
		attestlog_AD/.deps \
		attestlog_AD/.libs \
		attestlog_AD/Makefile
	mkdir -p m4
	touch m4/.keepdir
	rm -rf \
		configure~ \
		*.tar.gz
