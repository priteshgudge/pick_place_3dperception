## Project: Perception Pick & Place

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

Please see below.

### Exercise 1, 2 and 3 pipeline implemented
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.

[Here is a link to my python script for all exercises & the project.](https://github.com/priteshgudge/pick_place_3dperception/blob/master/pr2_robot/scripts/project_impl_script_2.py)



The pipeline is below:

Image -> Statistical Outlier Filter --> Voxel Grid Downsampling -> Passthrough over Z-Axis -> Passthrough over Y-Axis  ->
RANSAC PLANE Filter -> Done

Observations Below:

**Voxel Grid:** Leaf size is the area for the voxel grid. Interesting!

**Passthrough Filters:** It is important to note Right Hand Rule for the Axis. 

**Outlier Filter:** Straightforward with python_pcl

**RANSAC PLANE FILTER:** Plane intuition is easy to understand. Not sure how it will work for other shapes.

Here is what the cloud looked like after the pipeline:

![After Pipeline1](https://github.com/priteshgudge/pick_place_3dperception/raw/master/images/afterpipeline1.png)

#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.  

Euclidean clustering was done. A lot of trial and error was required to find hyperparameters.

Here is what the cloud looked like after the pipeline:

![After Pipeline2](https://github.com/priteshgudge/pick_place_3dperception/raw/master/images/afterpipeline2.png)

#### 3. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.

To create SVM:

Pictures: 800 per object

Bins (Color & Normals): 64

RGB vs HSV: RGB as HSV seemed to be overfitting.

Total Features: 6400


Here's the output of the training:

![Training](https://github.com/priteshgudge/pick_place_3dperception/raw/master/images/training.png)

I have a suspicion that the model is overfitting.

![Figure_1](https://github.com/priteshgudge/pick_place_3dperception/raw/master/images/figure_1.png)
![Figure_2](https://github.com/priteshgudge/pick_place_3dperception/raw/master/images/figure_2.png)

Here is the object and their markers:

![After Pipeline3](https://github.com/priteshgudge/pick_place_3dperception/raw/master/images/afterpipeline3.png)


### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

First, the results:

World 1 100%:

![World1](https://github.com/priteshgudge/pick_place_3dperception/raw/master/images/world1.png)

YAML File: [Link!](https://raw.githubusercontent.com/priteshgudge/pick_place_3dperception/master/pr2_robot/scripts/output_files/output_1.yaml)

World 2 100%:

![World2](https://github.com/priteshgudge/pick_place_3dperception/raw/master/images/world2.png)

YAML File: [Link!](https://raw.githubusercontent.com/priteshgudge/pick_place_3dperception/master/pr2_robot/scripts/output_files/output_2.yaml)

World 3 87.5%: Glue is incorrectly identified as biscuits.

![World3](https://github.com/priteshgudge/pick_place_3dperception/raw/master/images/world3.png)

YAML File: [Link!](https://raw.githubusercontent.com/priteshgudge/pick_place_3dperception/master/pr2_robot/scripts/output_files/output_3.yaml)

Now, my thoughts:
The project was quite challenging. I would have preferred more details into the algorithms/methods used for filtering.
