#! /bin/bash

# This script generates the Code Sourcery ARM EABI Toolchain on MAC OS X 10.7
# The new toolchain is installed at
# "$HOME/Developer/Cross/arm-cs-tools-$MENTOR_RELEASE-$TODAY"

# The implementation uses James Sneyder's Mafefile, available from
# https://github.com/jsnyder/arm-eabi-toolchain

# The build process also requires some special packages, that will be
# installed in a non-system folder, to avoid poluting the system

# There are many releases for Code Sourcery, and many releases of jsnyder's
# Makefile, so we need to identify them propely.

# The MENTOR_RELEASE is the internal calue used by Mentor, and 
# can be obtained from the Mentor URLs
# https://sourcery.mentor.com/sgpp/lite/arm/portal/release2032

if [ $# -gt 0 ] && [ $1 = "--help" ]
then
  echo
  echo "Script to build the Mentor/CodeSourcery ARM EABI toolchain on Mac OS X."
  echo
  echo "Usage:"
  echo "    $0 [clean]"
  echo "        Build the latest Mentor/CS version with the latest JS release."
  echo
  echo "    $0 2011.09-69 0084249 [clean]"
  echo "    $0 2011.03-49 0084249 [clean]"
  echo "    $0 2011.03-49 706e734 [clean]"
  echo "        Build the given Mentor/CS version with the given JS release."
  echo
  echo "Adding 'clean' to the end performs a 'make clean' on the given configuration."
  echo

  exit 0
fi

last=$(eval "echo \$$#")

if [ $# -gt 0 ]
then
  MENTOR_STRING=$1
else
  # Point to the latest Mentor/CodeSourcery release
  MENTOR_STRING="2011.09-69"
fi

if [ "$MENTOR_STRING" = "2011.09-69" ]
then
  # The Sourcery CodeBench Lite 2011.09-69, released 2011-12-19
  MENTOR_RELEASE=2032
elif [ "$MENTOR_STRING" = "2011.03-49" ]
then
  # The Sourcery G++ Lite 2011.03-42, released 2011-05-02
  MENTOR_RELEASE=1802
else
  echo "Unsupported Mentor/CS release $MENTOR_STRING"
  exit 1
fi

if [ $# -gt 1 ]
then
  JS_RELEASE=$2
else
  # Point to the latest CodeSourcery release
  JS_RELEASE="0084249"
fi

# Identify the James Sneyder Makefile Git version from the first 7 chars
if [ $JS_RELEASE = "0084249" ]
then
  JSNYDER_GIT_ID="00842493ba858c3bc45f6b01f239694f921f2a49"
elif [ $JS_RELEASE = "706e734" ]
then
  if [ MENTOR_RELEASE -gt 1802 ]
  then
    echo "JS release $JS_RELEASE supports Mentor/CS releases up to 2011.03-49"
    exit 1
  fi
  JSNYDER_GIT_ID="706e73495baecc7959d1a6768dd74178788dcdc5"
else
  echo "Unsupported JS release $JS_RELEASE"
  exit 1
fi

echo "Building Mentor/CS release $MENTOR_STRING with JS release $JS_RELEASE"

# Useful when testing
#TEST="-test"

# ----------------------------------------------------------------------------- 

patch_makefile () {

if [ $JS_RELEASE = "706e734" ]
then

# The quotes are used to avoid substitutions inside the string
(cat << "END_OF_FILE" | patch) || exit 1
--- Makefile	2011-12-13 18:10:22.000000000 +0200
+++ ../../arm-cs-tools-Makefile-1802-706e734.mk	2012-01-15 18:21:31.000000000 +0200
@@ -1,23 +1,25 @@
 SHELL = /bin/bash
 TARGET = arm-none-eabi
 PREFIX ?= $(HOME)/arm-cs-tools/
-PROCS = 4
+PROCS ?= 4
+
+CS_BASE		?= 2011.03
+CS_REV 		?= 42
+GCC_VERSION 	?= 4.5
+MPC_VERSION 	?= 0.8.1
+SOURCE_PACKAGE	?= 8733
+BIN_PACKAGE	?= 8734
 
-CS_BASE		= 2011.03
-CS_REV 		= 42
-GCC_VERSION 	= 4.5
-MPC_VERSION 	= 0.8.1
 CS_VERSION 	= $(CS_BASE)-$(CS_REV)
 
 LOCAL_BASE 	= arm-$(CS_VERSION)-arm-none-eabi
 LOCAL_SOURCE 	= $(LOCAL_BASE).src.tar.bz2
 LOCAL_BIN 	= $(LOCAL_BASE)-i686-pc-linux-gnu.tar.bz2
-SOURCE_URL 	= http://sourcery.mentor.com/sgpp/lite/arm/portal/package8733/public/arm-none-eabi/$(LOCAL_SOURCE)
-BIN_URL 	= http://sourcery.mentor.com/sgpp/lite/arm/portal/package8734/public/arm-none-eabi/$(LOCAL_BIN)
-
+SOURCE_URL 	= http://sourcery.mentor.com/sgpp/lite/arm/portal/package$(SOURCE_PACKAGE)/public/arm-none-eabi/$(LOCAL_SOURCE)
+BIN_URL 	= http://sourcery.mentor.com/sgpp/lite/arm/portal/package$(BIN_PACKAGE)/public/arm-none-eabi/$(LOCAL_BIN)
 
-SOURCE_MD5_CHCKSUM = 7c302162ec813d039b8388bd7d2b4176
-BIN_MD5_CHECKSUM = b1bd1dcb1f922d815ba7fa8d0e6fcd37
+SOURCE_MD5_CHCKSUM ?= 7c302162ec813d039b8388bd7d2b4176
+BIN_MD5_CHECKSUM ?= b1bd1dcb1f922d815ba7fa8d0e6fcd37
 
 install-cross: cross-binutils cross-gcc cross-g++ cross-newlib cross-gdb
 install-deps: gmp mpfr mpc
@@ -150,14 +152,14 @@
 	pushd ../../gcc-$(GCC_VERSION)-$(CS_BASE) ; \
 	make clean ; \
 	popd ; \
-	../../gcc-$(GCC_VERSION)-$(CS_BASE)/configure --prefix=$(PREFIX) --target=$(TARGET) --enable-languages="c" --with-gnu-ld --with-gnu-as --with-newlib --disable-nls --disable-libssp --with-newlib --without-headers --disable-shared --disable-threads --disable-libmudflap --disable-libgomp --disable-libstdcxx-pch --disable-libunwind-exceptions --disable-libffi --enable-extra-sgxxlite-multilibs && \
+	../../gcc-$(GCC_VERSION)-$(CS_BASE)/configure --prefix=$(PREFIX) --target=$(TARGET) $(DEPENDENCIES) --enable-languages="c" --with-gnu-ld --with-gnu-as --with-newlib --disable-nls --disable-libssp --with-newlib --without-headers --disable-shared --disable-threads --disable-libmudflap --disable-libgomp --disable-libstdcxx-pch --disable-libunwind-exceptions --disable-libffi --enable-extra-sgxxlite-multilibs && \
 	$(MAKE) -j$(PROCS) && \
 	$(MAKE) installdirs install-target && \
 	$(MAKE) -C gcc install-common install-cpp install- install-driver install-headers
 
 cross-g++: cross-binutils cross-gcc cross-newlib gcc-$(GCC_VERSION)-$(CS_BASE) multilibbash
 	mkdir -p build/g++ && cd build/g++ && \
-	../../gcc-$(GCC_VERSION)-$(CS_BASE)/configure --prefix=$(PREFIX) --target=$(TARGET) --enable-languages="c++" --with-gnu-ld --with-gnu-as --with-newlib --disable-nls --disable-libssp --with-newlib --without-headers --disable-shared --disable-threads --disable-libmudflap --disable-libgomp --disable-libstdcxx-pch --disable-libunwind-exceptions --disable-libffi --enable-extra-sgxxlite-multilibs && \
+	../../gcc-$(GCC_VERSION)-$(CS_BASE)/configure --prefix=$(PREFIX) --target=$(TARGET) $(DEPENDENCIES) --enable-languages="c++" --with-gnu-ld --with-gnu-as --with-newlib --disable-nls --disable-libssp --with-newlib --without-headers --disable-shared --disable-threads --disable-libmudflap --disable-libgomp --disable-libstdcxx-pch --disable-libunwind-exceptions --disable-libffi --enable-extra-sgxxlite-multilibs && \
 	$(MAKE) -j$(PROCS) && \
 	$(MAKE) installdirs install-target && \
 	$(MAKE) -C gcc install-common install-cpp install- install-driver install-headers
END_OF_FILE

elif [ $JS_RELEASE = "0084249" ]
then

# The quotes are used to avoid substitutions inside the string
(cat << "END_OF_FILE" | patch) || exit 1
--- Makefile	2012-01-18 18:06:14.000000000 +0200
+++ ../../Makefile	2012-01-21 18:30:30.000000000 +0200
@@ -50,7 +50,7 @@
 SOURCE_URL 	= http://sourcery.mentor.com/sgpp/lite/arm/portal/package$(SOURCE_PACKAGE)/public/arm-none-eabi/$(LOCAL_SOURCE)
 BIN_URL 	= http://sourcery.mentor.com/sgpp/lite/arm/portal/package$(BIN_PACKAGE)/public/arm-none-eabi/$(LOCAL_BIN)

-SOURCE_MD5_CHCKSUM = ebe25afa276211d0e88b7ff0d03c5345
+SOURCE_MD5_CHCKSUM ?= ebe25afa276211d0e88b7ff0d03c5345
 BIN_MD5_CHECKSUM ?= 2f2d73429ce70dfb848d7b44b3d24d3f
END_OF_FILE

else
  echo "No need to patch Makefile"
fi

}

# ----------------------------------------------------------------------------- 

# Numeric representation of today, like 20110704
TODAY=`date "+%Y%m%d"`

BUILD_FOLDER="arm-cs-tools-$MENTOR_STRING-$JS_RELEASE-$TODAY"

mkdir -p "$BUILD_FOLDER"

cd $BUILD_FOLDER

# Get jsnyder's Makefile

JSNYDER_URL="https://github.com/jsnyder/arm-eabi-toolchain/zipball/$JSNYDER_GIT_ID"

JSNYDER_ZIP_FOLDER="jsnyder-arm-eabi-toolchain-$JS_RELEASE"
JSNYDER_ZIP_FILE="$JSNYDER_ZIP_FOLDER.zip"

if [ ! -d "$JSNYDER_ZIP_FOLDER" ]
then
  # Fetch James Sneyder archive
  if [ ! -f "$JSNYDER_ZIP_FILE" ]
  then
    # Fetch $JSNYDER_ZIP_FILE
    curl -L $JSNYDER_URL -o "$JSNYDER_ZIP_FILE"
  fi

  # Unpack $JSNYDER_ZIP_FILE
  unzip "$JSNYDER_ZIP_FILE"
  
  pushd "$JSNYDER_ZIP_FOLDER"
  cp -a Makefile Makefile.orig  

  MAKEFILE_REFERENCE="../../Makefile-$JS_RELEASE.reference"
  if [ "$last" = "patch" ]
  then
    if [ -f "../../Makefile" ]
    then
      echo "Generating the patch file"
      diff -u Makefile "../../Makefile" >"../../Makefile.patch"
      mv ../../Makefile "$MAKEFILE_REFERENCE"
      popd
      echo "Check for Makefile.patch" 
      rm -rf "$JSNYDER_ZIP_FOLDER"
      exit 0
    else
      popd
      echo "The correct Makefile should be present in the current folder"
      exit 1
    fi
  else
    patch_makefile
    if [ -f "$MAKEFILE_REFERENCE" ]
    then
      echo "Comparing patched file with reference"
      diff "$MAKEFILE_REFERENCE" Makefile || exit 1
    fi
  fi
  popd
fi

# ----------------------------------------------------------------------------- 
# Use this to clean all build temporary folders

if [ $last = "clean" ]
then
  cd "$JSNYDER_ZIP_FOLDER"
  make clean
  exit 0
fi

# ----------------------------------------------------------------------------- 

# Keep different versions in different folders
TODAY_DESTINATION_FOLDER="$HOME/Developer/Cross/$BUILD_FOLDER"

# Append test
TODAY_DESTINATION_FOLDER+=$TEST

# Remove the destination folder, in case it exists
if [ -z "$TEST" ] && [ -d "$TODAY_DESTINATION_FOLDER" ]
then
  echo "Remove $TODAY_DESTINATION_FOLDER"
  rm -rf "$TODAY_DESTINATION_FOLDER"
fi


# ----------------------------------------------------------------------------- 
# Get Homebrew, if not already present
# DO NOT USE the default folder and do not link to /usr/local, to avoid
# interferences with other libraries

BREW_FOLDER=/brew

if [ ! -d "$BREW_FOLDER/local" ]
then
  echo "If asked, enter your sudo password to create the $BREW_FOLDER/local folder"
  sudo mkdir -p "$BREW_FOLDER/local"
  sudo chown -R $USER "$BREW_FOLDER/local"
  curl -Lsf http://github.com/mxcl/homebrew/tarball/master | tar xz --strip 1 -C$BREW_FOLDER/local
fi

if [ "$JS_RELEASE" = "0084249" ]
then
  BREW_REQUIRED="mpfr gmp libmpc libelf texinfo"
elif [ "$JS_RELEASE" = "706e734" ]
then
  BREW_REQUIRED="mpfr gmp libmpc texinfo"
fi

BREW_INSTALLED=$($BREW_FOLDER/local/bin/brew list)

for l in $BREW_REQUIRED
do
  BREW_HAS_LIB=`echo $BREW_INSTALLED | grep $l`
  if [ -z "$BREW_HAS_LIB" ]
  then
    $BREW_FOLDER/local/bin/brew install --env=std $l
  else
    $BREW_FOLDER/local/bin/brew upgrade --env=std $l
  fi
done

echo "Homebrew installed packages:"
$BREW_FOLDER/local/bin/brew list

if [ -z `$BREW_FOLDER/local/bin/brew list | grep mpfr` ]
then
  echo "Missing Homebrew mpfr, build not possible"
  exit 1
fi
# Homebrew should be fine now

# ----------------------------------------------------------------------------- 
# Prepare the make environment

# Define extra dependencies, in case they are not in the system path 
DEPENDENCY_DIR=$BREW_FOLDER/local
if [ "$JS_RELEASE" = "0084249" ]
then
  export DEPENDENCIES="--with-mpc=$DEPENDENCY_DIR --with-mpfr=$DEPENDENCY_DIR --with-gmp=$DEPENDENCY_DIR --with-libelf=$DEPENDENCY_DIR"
elif [ "$JS_RELEASE" = "706e734" ]
then
  export DEPENDENCIES="--with-mpc=$DEPENDENCY_DIR --with-mpfr=$DEPENDENCY_DIR --with-gmp=$DEPENDENCY_DIR"
fi

mkdir -p "$TODAY_DESTINATION_FOLDER/bin"

# Allow the build to use the newly generated tools to compile the libraries
export PATH=$TODAY_DESTINATION_FOLDER/bin:$PATH

# Pass the custom destination folder to the make process
export PREFIX=$TODAY_DESTINATION_FOLDER/
echo "PREFIX=$TODAY_DESTINATION_FOLDER/"

# Request to be as close as possible to CodeSourcery configuration
export MATCH_CS=true

# Set the number of available cores
export PROCS=`sysctl hw.ncpu | awk '{print $2}'`

# ----------------------------------------------------------------------------- 
# Customise for various releases

if [ $MENTOR_RELEASE = 2032 ]
then
  export CS_BASE=2011.09
  export CS_REV=69
  export GCC_VERSION=4.6
  export MPC_VERSION=0.8.1
  export SOURCE_PACKAGE=9739
  export BIN_PACKAGE=9740
  export SOURCE_MD5_CHCKSUM=ebe25afa276211d0e88b7ff0d03c5345
  export BIN_MD5_CHECKSUM=2f2d73429ce70dfb848d7b44b3d24d3f
elif [ $MENTOR_RELEASE = 1802 ]
then
  export CS_BASE=2011.03
  export CS_REV=42
  export GCC_VERSION=4.5
  export MPC_VERSION=0.8.1
  export SOURCE_PACKAGE=8733
  export BIN_PACKAGE=8734
  export SOURCE_MD5_CHCKSUM=7c302162ec813d039b8388bd7d2b4176
  export BIN_MD5_CHECKSUM=b1bd1dcb1f922d815ba7fa8d0e6fcd37
fi

# ----------------------------------------------------------------------------- 
# And finally start the build

cd "$JSNYDER_ZIP_FOLDER"

# Build the tools, using the new Xcode 4.1 compiler
# Leave the newlib at the end, since it might fail :-(


if [ $JS_RELEASE = "706e734" ]
then
  time ( \
    (CC=clang make cross-binutils cross-gcc cross-g++ cross-newlib) \
    && (make cross-gdb) \
    && (make install-bin-extras) \
  )
elif [ "$JS_RELEASE" = "0084249" ]
then
  time ( \
    (CC=clang make cross-binutils cross-gcc cross-newlib) \
    && (make cross-gdb) \
    && (make install-bin-extras) \
  )
fi

echo
echo "If you need to manually run various build steps..."
echo
echo cd `pwd`
echo export DEPENDENCIES=\"$DEPENDENCIES\"
echo export PATH=$PATH
echo export PREFIX=$PREFIX
echo export PROCS=$PROCS
echo export MATCH_CS=true
echo

exit 0
