
1. Use labelImg: https://github.com/tzutalin/labelImg
to create annotations for your object of interest


2. ./train_FFLDDetector --name chair --result chair_model.txt --nb-negatives 200 --annotation-dir /path/to/AnnotationDir /path/to/JPEGImages

3. ./test_FFLDDetector ./chair_model.txt /path/to/test/img


