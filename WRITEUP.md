## Project: Perception Pick & Place

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Exercise 1, 2 and 3 pipeline implemented
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.

[Here is a link to my python script for all exercises & the project.](https://github.com/priteshgudge/pick_place_3dperception/blob/master/pr2_robot/scripts/)



My pipeline is as follows:

Image -> Voxel Grid Downsampling -> Passthrough over Z-Axis -> Passthrough over X-Axis -> Outlier Filter ->
RANSAC PLANE Filter -> Done

For each of these sections, I'll list my observactions:

**Voxel Grid:** The hardest thing to learn here was what the LEAF_SIZE actually meant. I kept thinking it meant that
something like "1" meant one point in each "area." I didn't realize that the leaf is actually defining the "area."  

**Passthrough Filters:** The biggest killer to me on this one was remembering the right hand rule when I was learning
it.  This one is pretty easy, but will be my biggest challenge when I take the challenge_world.

**Outlier Filter:** After the working python_pcl was given to us, this one is so simple that it isn't worth mentioning.

**RANSAC PLANE FILTER:** This one still perplexes me.  I understand the concept, just not really how it works, I need
to mess with this one some more.

Here is what the cloud looked like after the pipeline:

![After Pipeline1](https://github.com/tiedyedguy/RoboND-Perception-Project/raw/master/images/afterpipeline1.png)

#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.  

I used the same Euclidean Clutering that was given to us a sample.  For the project as opposed to the exercises, 
I had to mess around with the values a lot more until I got things that worked.

Here is what the cloud looked like after the pipeline:

![After Pipeline2](https://github.com/tiedyedguy/RoboND-Perception-Project/raw/master/images/afterpipeline2.png)

#### 3. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.

Here are the settings I used to create my SVM:

Pictures: 800

Bins (Color & Normals): 64

RGB vs HST: RGB

Total Features: 6400

Here's the output of the training:

![Training](https://github.com/priteshgudge/pick_place_3dperception/raw/master/images/training.png)
![Figure_1](https://github.com/priteshgudge/pick_place_3dperception/raw/master/images/figure_1.png)
![Figure_2](https://github.com/priteshgudge/pick_place_3dperception/raw/master/images/figure_2.png)

Here is the object and their markers:

![After Pipeline3](https://github.com/priteshgudge/pick_place_3dperception/raw/master/images/afterpipeline3.png)


### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

First, the results:

World 1 100%:

![World1](https://github.com/tiedyedguy/RoboND-Perception-Project/raw/master/images/world1.png)

YAML File: [Link!](https://raw.githubusercontent.com/priteshgudge/pick_place_3dperception/master/pr2_robot/scripts/output_1.yaml)

World 2 100%:

![World2](https://github.com/priteshgudge/pick_place_3dperception/raw/master/images/world2.png)

YAML File: [Link!](https://raw.githubusercontent.com/priteshgudge/pick_place_3dperception/master/pr2_robot/scripts/output_2.yaml)

World 3 87.5%:

![World3](https://github.com/priteshgudge/pick_place_3dperception/raw/master/images/world3.png)

YAML File: [Link!](https://raw.githubusercontent.com/priteshgudge/pick_place_3dperception/master/pr2_robot/scripts/output_3.yaml)

Now, my thoughts:
 The only major challenge I found was upping
the number of features from 500 to 6400 in the training.  This got me 100%  accuracy in world 1, 100% accuracy in world 2 and 87.5% accuracy in World 3

