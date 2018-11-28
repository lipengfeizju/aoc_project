                          Implementation of the papers

                "Exact Acceleration of Linear Object Detectors"
               12th European Conference on Computer Vision, 2012.

             "Deformable Part Models with Individual Part Scaling"
                  24th British Machine Vision Conference, 2013.

      Copyright (c) 2013 Idiap Research Institute, <http://www.idiap.ch/>
              Written by Charles Dubout <charles.dubout@idiap.ch>


                                 PREREQUISITES

FFLD requires that various libraries be available for use in the build
procedure.

Eigen version 3.1 or later
  Eigen is a C++ template library for linear algebra: matrices, vectors,
  numerical solvers, and related algorithms.
  Download it at <http://eigen.tuxfamily.org/> and make sure it is accessible by
  your compiler.
  Alternatively you can simply uncompress the downloaded Eigen archive into the
  FFLD folder.

Libfftw3f (or libfftw3 if the option FFLD_HOGPYRAMID_DOUBLE is on)
  A C subroutine library for computing the discrete Fourier transform (DFT) in
  one or more dimension, of arbitrary input size, and of both real and complex
  complex data (as well as of even/odd data, i.e. the discrete cosine/sine
  transforms or DCT/DST). 
  There are no CMake FFTW package, so CMakeLists.txt looks for it in the
  standard paths.

Libjpeg
  A widely used C library for reading and writing JPEG image files.
  Found automatically using CMake FIND_PACKAGE(JPEG).

Libxml2
  The XML C parser and toolkit developed for the Gnome project (but usable
  outside of the Gnome platform), it is free software available under the MIT
  License.
  Found automatically using CMake FIND_PACKAGE(LibXml2).

OpenMP
  An API for multi-platform shared-memory parallel programming in C/C++ and
  Fortran.
  Found automatically using CMake FIND_PACKAGE(OpenMP).
  Not required, but stronlgy recommended on multi-core systems to take
  advantage of multithreading.
  This is not only a library but also a compiler extension, so refer to your
  compiler documentation for any information.

SimpleOpt
  A cross-platform command line library which can parse almost any of the
  standard command line formats in use today.
  Already included in the FFLD archive.


                                 CONFIGURATION

A few options can be configured before building FFLD. None are enabled by
default. To modify them, run ccmake or simply replace OFF by ON in the file
CMakeLists.txt.

FFLD_HOGPYRAMID_DOUBLE
  Use doubles instead of floats (slower, uses twice more memory, and the
  increase in precision is not necessarily useful).

FFLD_HOGPYRAMID_EXTRA_FEATURES
  Use extra features (LBP + color) in addition to HOG.

FFLD_MODEL_3D
  Allow parts to also deform across scales.

FFLD_MIXTURE_STANDARD_CONVOLUTION
  Use standard convolutions instead of the optimized Fourier ones.


                                    BUILDING

If you do not already have Eigen, below is the quickest way to get it

  cd ffld_v2
  wget http://bitbucket.org/eigen/eigen/get/3.2.0.tar.gz
  tar xzf 3.2.0.tar.gz
  mv eigen-*/Eigen .
  rm -rf eigen-*

The last 2 steps are optional, they simply delete useless files.
It is recommended to build the software in a separate directory. For example

  cd ffld_v2
  mkdir build
  cd build

Then use CMake to generate the necessary Makefiles

  cmake ..

Then build the software with

  make


                               TESTING (optional)

Run

  ./test --model ../models/bicycle_2d.txt --images . --threshold=0 ../bicycle.jpg

If the software was built successfully the current folder should contain a
bicycle.jpg image where the bicycle was detected.


                                  INTRODUCTION

The train executable can be used to train a deformable part-based model on a
Pascal VOC dataset.
The test executable can be used to run a deformable part-based model either on
an image or on a Pascal VOC dataset.

The first time you run it it will be slow as the FFTW library will search for
the best plans using runtime measurements. The resulting plans will then be
saved to a file named wisdom.fftw and reused in the future.


                              COMMAND LINE OPTIONS

After building the train and test executables you can run them without any
argument to get a list of all the possible parameters.

  -m,--model <file>
  Read the input model from <file> (default "" for train, "model.txt" for test)

The models are stored in a text file format with the following grammar (an
example can be found in the file bicycle.txt)

Mixture := nbModels Model*
Model := nbParts bias Part*
Part := nbRows nbCols nbFeatures xOffset yOffset zOffset a b c d e f value*

Where nbModels is the number of mixture components (models); nbParts is the
number of parts (including the root) in the model; bias is the offset to add to
the scores of the model; nbRows, nbCols, nbFeatures are the dimensions of the
part filter; xOffset, yOffset, zOffset are the offsets of the part relative to
the root (anchor); a, b, c, d, e, f are the deformation coefficients
(ax^2 + bx + cy^2 + dy + ez^2 + fz); values are the filter coefficients, stored
in row-major order, and of size nbRows x nbCols x nbFeatures.

In the current implementation nbFeatures must be 32, the number of HOG features
(or 48 if FFLD was compiled with FFLD_HOGPYRAMID_EXTRA_FEATURES=ON).
One can use the provided Matlab script 'convertmodel4.m' to convert to this
format the models of P. Felzenszwalb, R. Girshick and D. McAllester.
Discriminatively Trained Deformable Part Models, Release 4.
http://people.cs.uchicago.edu/~pff/latent-release4/.
One can use the provided Matlab script 'convertmodel5.m' to convert to this
format the models of P. Felzenszwalb, R. Girshick and D. McAllester.
Discriminatively Trained Deformable Part Models, Release 5.
http://people.cs.uchicago.edu/~rbg/latent-release5/.

  -n,--name <arg>
  Name of the object to train or detect (default "person")

  -r,--results <file>
  Write the trained model or the detection results to <file> (default
  "model.txt" for train, "" for test)

The test executable can outputs the list of all the detections into a file, in
the format of the Pascal VOC challenge (one line by detection, and for each
detection the scene id, the score and the bounding box: xmin, ymin, xmax, ymax).

  -i,--images <folder>
  Draw the detections to <folder> (default none)

The test executable can also output images with the detections drawn. In that
case it might be useful to set a higher detection threshold so as to only draw
detection with a high enough score.

  -z,--nb-negatives <arg>
  Maximum number of negative images to consider (default all)

It might be useful to run the detector only on a reduced number of negative
(background) Pascal VOC scenes in order to save time while training or
evaluating the performance of a detector.

  -p,--padding <arg>
  Amount of zero padding in HOG cells (default 6)

Must be greater or equal to half the greatest filter dimension.

  -e,--interval <arg>
  Number of levels per octave in the HOG pyramid (default 5)

  -t,--threshold <arg>
  Minimum detection threshold (default -10)

To set a negative threshold you need to use the option as in -t=-1

  -v,--overlap <arg>
  Minimum overlap in in latent positive search and non maxima suppression
  (default 0.7 for train, 0.5 for test)

  -x,--nb-components <arg>
  Number of mixture components (without symmetry, default 3).

  -l,--relabel <arg>
  Maximum number of training iterations (default 8, half if the model has no
  part).

  -d,--datamine <arg>
  Maximum number of data-mining iterations within each training iteration
  (default 10).

  -c,--C <arg>
  SVM regularization constant (default 0.002).

  -j,--J <arg>
  SVM positive regularization constant boost (default 2).

  -s,--seed <arg>
  Random seed (default time(NULL)).


                                    EXAMPLES

To train or evaluate a model on a Pascal VOC dataset you must pass to the
executable the corresponding image set file. It will look for the Pascal
annotations in the folder 'Annotations' two levels below ("../../Annotations/")
and for the jpeg images in the folder 'JPEGImages' two levels below
("../../JPEGImages/").

A complete example for train could be

  ./train --name bicycle --result bicycle.txt --nb-negatives 200 VOC2007/ImageSets/Main/bicycle_trainval.txt
  ./train --model bicycle.txt --name bicycle --result bicycle.txt --nb-negatives 2000 --relabel 1 VOC2007/ImageSets/Main/bicycle_trainval.txt

Which takes ~1 hour to complete on my laptop.

While a complete example for test could be

  ./test --model ../models/bicycle_2d.txt --images . --threshold=-0.5 VOC2007/ImageSets/Main/bicycle_test.txt

Which takes ~15 minutes to complete on my laptop.


                                     MODELS

The "models" subfolder contains 2D and 3D models trained by this version of
FFLD. The *_init.txt files are the starting points for both kinds of DPM.
Also note that during training after every iterations the latest model is
continuously saved in the file "tmp.txt".
