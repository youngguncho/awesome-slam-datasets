## Awesome SLAM Datasets

This repository is the collection of SLAM-related datasets. Among various SLAM datasets, we've selected the datasets provide pose and map information.

We provide several category for each access of the data.

## TODO
This repository is currently working on.
Target caterory and project page will be on next.


## Caetegory
- [Platform](#categorized-by-platform)
    - [Vehicle (Veh)](#vehicle): Commercial Vehicle (Four-wheeled on the road)
    - [Mobile robot (Mob)](#mobile-robot): Mobile Robots (Ex. Husky, Rover.. )
    - [Unmanned Aerial Vehicle (UAV)](#unmanned-aerial-vehicle): Unmanne aerial robots include drone.
    - [Autonomous Underwater Vehicle (AUV)](#autonomous-underwater-vehicle): Underwater robots include ROV for simplicity.
    - [Unmanned Surface Vehicle (USV)](#unmanned-surface-vehicle): Water surface vehicle such as canoe and boat.
    - [Hand-held Device (Hand)](#hand-held-device): Hand-held platform by human

- [Envoriment](#categorized-by-platform)
    - [Urban](#urban): City, campus, town, and infrastructures
    - [Indoor](#indoor): Indoor environement
    - [Terrain](#terrain): Rough terrain, underground, lake and farm
    - [Underwater](#underwater): Underwater

## Overall datasets chart (Simplified Version)

| Shotname                  | Affiliation  | Year | Platform   | Publication | Environment           | GT - Pose | GT - Map | IMU | GPS | Labels | Lidar      | Vision | RGBD | Event | Radar | Sonar |
|---------------------------|--------------|------|------------|-------------|-----------------------|-----------|----------|-----|-----|--------|------------|--------|------|-------|-------|-------|
| [FABMAP](http://www.robots.ox.ac.uk/~mobile/IJRR_2008_Dataset/)                    | Oxford-Robot | 2008 | Veh        | IJRR        | Urban                 |           |          |     | O   |        |            | O      |      |       |       |       |
| [COLD](https://www.pronobis.pro/#data)                      | KTH          | 2009 | Hand       | IJRR          | Indoor                | O         |          |     |     | O      | O          | O      |      |       |       |       |
| [NewCollege](http://www.robots.ox.ac.uk/NewCollegeData/)                | Oxford-Robot | 2009 | Mob        | IJRR        | Urban                 | O         |          |     | O   |        | O          | O      |      |       |       |       |
| [MIT-DARPA](http://grandchallenge.mit.edu/wiki/index.php?title=PublicData)           | MIT          | 2010 | Veh        | IJRR        | Urban                 | O         |          | O   | O   | O      | O          | O      |      |       |       |       |
| [St Lucia Stereo](http://asrl.utias.utoronto.ca/~mdw/uqstluciadataset.html)           | UToronto     | 2010 | Veh        | ACRA        | Urban                 |           |          | O   | O   |        |            | O      |      |       |       |       |
| [St Lucia Multiple Times](https://wiki.qut.edu.au/display/cyphy/St+Lucia+Multiple+Times+of+Day)   | QUT          | 2010 | Veh        | ICRA        | Urban                 |           |          |     | O   |        |            | O      |      |       |       |       |
| [Marulan](http://sdi.acfr.usyd.edu.au/)                   | ACFR         | 2010 | Mob        | IJRR        | Terrain               | O         |          | O   | O   |        | O          | O      |      |       | O     |       |
| [UTIAS Multi-Robot](http://asrl.utias.utoronto.ca/datasets/mrclam/)         | UT-IAS       | 2011 | Mob        | IJRR        | Urban                 | O         |          |     |     | O      |            |        |      |       |       |       |
| [Ford Campus](http://robots.engin.umich.edu/SoftwareData/Ford)               | UMich        | 2011 | Veh        | IJRR        | Urban                 | O         |          | O   | O   | O      | O          | O      |      |       |       |       |
| [San francisco](https://sites.google.com/site/chenmodavid/datasets)             | Stanford     | 2011 | Veh        | CVPR        | Urban                 | O         |          | O   | O   | O      |            | O      |      |       |       |       |
| [Annotated-laser](http://any.csie.ntu.edu.tw/data)           | NTU          | 2011 | Veh        | IJRR        | Urban                 | O         |          |     |     | O      | O          | O      |      |       |       |       |
| [SeqSLAM](https://wiki.qut.edu.au/display/cyphy/Open+datasets+and+software)                   | QUT          | 2012 | Veh        |  ICRA       | Urban                 |           |          |     |     | O      |            | O      |      |       |       |       |
| [ETH-challenging](http://projects.asl.ethz.ch/datasets/doku.php?id=laserregistration:laserregistration)           | ETH-ASL      | 2012 | Hand       |  IJRR       | Urban / Terrain       |           |          | O   | O   |        | O          | O      | O    |       |       |       |
| [TUM-RGBD](https://vision.in.tum.de/data/datasets/rgbd-dataset)                  | TUM          | 2012 | Hand / Mob |  IROS       | Indoor                | O         |          | O   |     |        |            |        | O    |       |       |       |
| [ASRL-Kagara-airborn](http://asrl.utias.utoronto.ca/~mdw/kagarudataset.html)       | UToronto     | 2012 | UAV        |   FSR      | Terrain               |           |          | O   | O   |        |            | O      |      |       |       |       |
| [Devon Island Rover](http://asrl.utias.utoronto.ca/datasets/devon-island-rover-navigation/)        | UToronto     | 2012 | Mob        |   IJRR      | Terrain               | O         |          |     | O   |        |            | O      |      |       |       |       |
| [ACFR Marine](http://marine.acfr.usyd.edu.au/datasets/)               | ACFR         | 2012 | AUV        |             | Underwater            | O         |          | O   |     | O      |            | O      |      |       |       | O     |
| [KITTI](http://www.cvlibs.net/datasets/kitti/index.php)                     | KIT          | 2013 | Veh        | IJRR        | Urban                 | O         |          | O   | O   | O      | O          | O      |      |       |       |       |
| [Canadian Planetary](http://asrl.utias.utoronto.ca/datasets/3dmap/#Datasets)        | UToronto     | 2013 | Mob        |  IJRR      | Terrain               | O         |          | O   | O   |        | O (sensor) | O      |      |       |       |       |
| [ICL-NUIM](https://www.doc.ic.ac.uk/~ahanda/VaFRIC/iclnuim.html)                  | Imperial     | 2014 | Hand       |  ICRA       | Indoor                | O         | O        |     |     |        |            |        | O    |       |       |       |
| [MRPT-Malaga](https://www.mrpt.org/robotics_datasets)               | MRPT         | 2014 | Veh        | AR          | Urban                 |           |          | O   | O   |        | O          | O      |      |       |       |       |
| [CCSAD](http://aplicaciones.cimat.mx/Personal/jbhayet/ccsad-dataset)                     | CIMAT        | 2015 | Veh        | CAIP        | Urban                 |           |          | O   | O   |        |            | O      |      |       |       |       |
| [Augmented ICL-NUIM](http://redwood-data.org/indoor/index.html)        | Redwood      | 2015 | Hand       |  CVPR       | Indoor                | O         | O        |     |     |        |            |        | O    |       |       |       |
| [EuRoc](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)                     | ETH-ASL      | 2016 | UAV        |    IJRR    | Indoor                | O         | O        | O   |     |        |            | O      |      |       |       |       |
| [Cityscape](https://www.cityscapes-dataset.com/)                 | Daimler AG   | 2016 | Veh        | CVPR        | Urban                 | O         |          |     | O   | O      |            | O      |      |       |       |       |
| [Solar-UAV](http://projects.asl.ethz.ch/datasets/doku.php?id=fsr2015)                 | ETHZ         | 2016 | UAV        |  CVPR       | Terrain               | O         | O        | O   | O   |        | O          |        |      |       |       |       |
| [Rawseeds-indoor](http://www.rawseeds.org/home/category/benchmarking-toolkit/datasets/)           | Milano       | 2009 | Mob        |   IROSW      | Indoor                | O         | O        | O   |     |        | O          | O      |      |       |       | O     |
| [Rawseeds-outdoor](http://www.rawseeds.org/home/category/benchmarking-toolkit/datasets/)          | Milano       | 2009 | Mob        |   IROSW     | Urban                 | O         | O        | O   | O   |        | O          | O      |      |       |       | O     |
| [CoRBS](http://corbs.dfki.uni-kl.de/?pagerd_tumlltzzf42zsv6de7b9)                     | DFKI         | 2016 | Hand       |   WACV     | Indoor                | O         | O        |     |     |        |            |        | O    |       |       |       |
| [Oxford-robotcar](http://robotcar-dataset.robots.ox.ac.uk)           | Oxford       | 2016 | Veh        | IJRR        | Urban                 | O         |          | O   | O   |        | O          | O      |      |       |       |       |
| [NCLT](http://robots.engin.umich.edu/nclt/)                      | UMich        | 2016 | Mob        | IJRR        | Urban                 | O         |          | O   | O   |        | O          |        |      |       |       |       |
| [RPG-event](http://rpg.ifi.uzh.ch/davis_data.html)                 | ETH-RPG      | 2017 | UAV / Hand |  IJRR       | Indoor                | O         |          | O   |     |        |            | O      |      | O     |       |       |
| [MPO-Japan](http://robotics.ait.kyushu-u.ac.jp/kurazume_lab/research-e.php?content=db) | Kyushu U     | 2016 | Veh        | IROS     | Urban / Terrain       |           |          | O   | O   |        | O          | O      |      |       |       |       |
| [Underwater Cave](http://cirs.udg.edu/caves-dataset/)           | UDG          | 2017 | AUV        | IJRR        | Underwater            | O         |          | O   |     |        |            | O      |      |       |       | O     |
| [Robot @ Home](http://mapir.isa.uma.es/mapirwebsite/index.php/mapir-downloads/203-robot-at-home-dataset)              | MRPT         | 2017 | Mob        |  IJRR     | Indoor                | O         | O        |     |     | O      | O          |        | O    |       |       |       |
| [Zurich Urban MAV](http://rpg.ifi.uzh.ch/zurichmavdataset.html)          | ETH-RPG      | 2017 | UAV        | IJRR        | Urban                 | O         |          | O   | O   |        |            | O      |      |       |       |       |
| [Chilean Underground](http://dataset.amtc.cl/#)       | Trimble      | 2017 | Mob        |    IJRR         | Terrain (Underground) | O         |          |     |     |        | O          | O      |      |       | O     |       |
| [SceneNet RGB-D](https://robotvault.bitbucket.io/scenenet-rgbd.html)            | Imperial     | 2017 | Hand       |    ICCV         | Indoor                | O         |          |     |     | O      |            |        | O    |       |       |       |
| [Symphony Lake](http://dream.georgiatech-metz.fr/?q=node/79)             | Georgia Tech | 2017 | USV        |     IJRR        | Terrain (Lake)        |           |          | O   | O   |        | O          | O      |      |       |       |       |
| [Agricultural robot](http://www.ipb.uni-bonn.de/data/sugarbeets2016/)        | Bonn         | 2017 | Mob        |     IJRR        | Terrain               | O         |          |     | O   | O      | O          | O      | O    |       |       |       |
| [Beach Rover](https://robotics.estec.esa.int/datasets/katwijk-beach-11-2015/)               | TEC-MMA      | 2018 | Mob        |      IJRR       | Terrain               | O         |          | O   | O   |        | O          | O      | O    |       |       |       |
| [KAIST Day/Night](https://sites.google.com/view/multispectral/home)           | KAIST-RCV    | 2018 | Veh        | T-ITS       | Urban                 | O         |          | O   | O   | O      | O          | O      |      |       |       |       |
| [Multi Vech Event](https://daniilidis-group.github.io/mvsec/)          | Upenn        | 2018 | Veh        | RA-L        | Urban                 | O         |          | O   | O   |        | O          | O      |      | O     |       |       |
| [VI Canoe](https://databank.illinois.edu/datasets/IDB-9342111)                  | UIUC         | 2018 | USV        |     IJRR        | Terrain               | O         |          | O   | O   |        |            | O      |      |       |       |       |
| [Complex Urban](http://irap.kaist.ac.kr/dataset/)               | KAIST-IRAP   | 2018 | Veh        | ICRA        | Urban                 | O         | O        | O   | O   |        | O          |        |      |       |       |       |

## Categorized by Platform

### Vehicle

- [FABMAP Dataset](http://www.robots.ox.ac.uk/~mobile/IJRR_2008_Dataset/)
- [MIT DARPA Urban Challenge Dataset](http://grandchallenge.mit.edu/wiki/index.php?title=PublicData)
- [St Lucia Stereo Vehicular Dataset](http://asrl.utias.utoronto.ca/~mdw/uqstluciadataset.html)
- [St Lucia Multiple Times of Day](https://wiki.qut.edu.au/display/cyphy/St+Lucia+Multiple+Times+of+Day)
- [Ford Campus Vision and Lidar Dataset](http://robots.engin.umich.edu/SoftwareData/Ford)
- [San Francisco Landmark Dataset](https://sites.google.com/site/chenmodavid/datasets)
- [Annotated-laser Dataset](http://any.csie.ntu.edu.tw/data) (Link Broken)
- [Alderley Day/Night Dataset](https://wiki.qut.edu.au/pages/viewpage.action?pageId=181178395)
- [Day and Night with Lateral Pose Change Dataset](https://wiki.qut.edu.au/display/cyphy/Day+and+Night+with+Lateral+Pose+Change+Datasets)
- [KITTI Vision Benchmark Suite](http://www.cvlibs.net/datasets/kitti/index.php)
- [Málaga Stereo and Laser Urban Data Set](https://www.mrpt.org/MalagaUrbanDataset)
- [CCSAD (Stereo Urban) Dattaset](http://aplicaciones.cimat.mx/Personal/jbhayet/ccsad-dataset)
- [CityScapes Dataset](https://www.cityscapes-dataset.com/)
- [Oxford Robotcar Dataset](http://robotcar-dataset.robots.ox.ac.uk/)
- [Multi-modal Panoramic 3D Outdoor Dataset (MPO)](http://robotics.ait.kyushu-u.ac.jp/kurazume_lab/research-e.php?content=db)
- [KAIST Day/Night Dataset](https://sites.google.com/view/multispectral/home)
- [Multi Vehicle Stereo Event Camera Dataset](https://docs.google.com/spreadsheets/d/1mudM7LxXv09ywuQGDp3t_RlIjIdwzg_ZaMu78agLmH4/edit#gid=0)
- [Complex Urban Dataset](http://irap.kaist.ac.kr/dataset/)


### Mobile Robot
- [New College Vision and Laser Data Set](http://www.robots.ox.ac.uk/NewCollegeData/)
- [TUM RGB-D SLAM Dataset and Benchmark](https://vision.in.tum.de/data/datasets/rgbd-dataset)
- [Multi-Sensor Perception (Marulan) Dataset ](http://sdi.acfr.usyd.edu.au/)
- [Multi-Robot Cooperative Localization and Mapping Dataset](http://asrl.utias.utoronto.ca/datasets/mrclam/)
- [Devon Island Rover Navigation Dataset](http://asrl.utias.utoronto.ca/datasets/devon-island-rover-navigation/)
- [Canadian Planetary Emulation Terrain 3D Mapping Dataset](http://asrl.utias.utoronto.ca/datasets/3dmap/#Datasets)
- [Rawseeds In/Outdoor Dataset](http://www.rawseeds.org/home/category/benchmarking-toolkit/datasets/)
- [University of Michigan North Campus Long-Term (NCLT) Vision and LIDAR Dataset](http://robots.engin.umich.edu/nclt/)
- [Robot @ Home Dataset](http://mapir.isa.uma.es/mapirwebsite/index.php/mapir-downloads/203-robot-at-home-dataset)
- [Chilean Underground Mine Dataset](http://dataset.amtc.cl/#)
- [Sugar Beets 2016, Agricultural Robot Dataset](http://www.ipb.uni-bonn.de/data/sugarbeets2016/)
- [Katwijk Beach Planetary Rover Dataset](https://robotics.estec.esa.int/datasets/katwijk-beach-11-2015/)


### Unmanned Aerial Vehicle
- [Kagaru Airborne Stereo Dataset Dataset](http://asrl.utias.utoronto.ca/~mdw/kagarudataset.html)
- [EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)
- [Solar-powered UAV Sensing and Mapping Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=fsr2015)
- [Event-Camera Dataset and Simulator](http://rpg.ifi.uzh.ch/davis_data.html)
- [Zurich Urban Micro Aerial Vehicle Dataset](http://rpg.ifi.uzh.ch/zurichmavdataset.html)


### Autonomous Underwater Vehicle
- [ACFR Marine Robotics Dataset](http://marine.acfr.usyd.edu.au/datasets/)
- [Underwater Caves SONAR and Vision Dataset](http://cirs.udg.edu/caves-dataset/)


### Unmanned Surface Vehicle
- [Symphony Lake Dataset](http://dream.georgiatech-metz.fr/?q=node/79)
- [Visual-Inertial Canoe Dataset](https://databank.illinois.edu/datasets/IDB-9342111)


### Hand-held Device
- [Cosy Localization Database (COLD)](https://www.pronobis.pro/#data)
- [Challenging data sets for point cloud registration algorithms](https://projects.asl.ethz.ch/datasets/doku.php?id=laserregistration:laserregistration)
- [ICL-NUIM RGBD Dataset](https://www.doc.ic.ac.uk/~ahanda/VaFRIC/iclnuim.html)
- [Augmented ICL-NUIM Reconstruction Dataset](http://redwood-data.org/indoor/index.html)
- [Comprehensive RGB-D Benchmark (CoRBS)](http://corbs.dfki.uni-kl.de/?pagerd_tumlltzzf42zsv6de7b9)
- [Event-Camera Dataset and Simulator](http://rpg.ifi.uzh.ch/davis_data.html)
- [SceneNet RBG-D Dataset](https://robotvault.bitbucket.io/scenenet-rgbd.html)


## Categorized by Environment
### Urban
- [FABMAP Dataset](http://www.robots.ox.ac.uk/~mobile/IJRR_2008_Dataset/)
- [New College Vision and Laser Data Set](http://www.robots.ox.ac.uk/NewCollegeData/)
- [MIT DARPA Urban Challenge Dataset](http://grandchallenge.mit.edu/wiki/index.php?title=PublicData)
- [St Lucia Stereo Vehicular Dataset](http://asrl.utias.utoronto.ca/~mdw/uqstluciadataset.html)
- [St Lucia Multiple Times of Day](https://wiki.qut.edu.au/display/cyphy/St+Lucia+Multiple+Times+of+Day)
- [Multi-Robot Cooperative Localization and Mapping Dataset](http://asrl.utias.utoronto.ca/datasets/mrclam/)
- [Ford Campus Vision and Lidar Dataset](http://robots.engin.umich.edu/SoftwareData/Ford)
- [San Francisco Landmark Dataset](https://sites.google.com/site/chenmodavid/datasets)
- [Annotated-laser Dataset](http://any.csie.ntu.edu.tw/data) (Link Broken)
- [Alderley Day/Night Dataset](https://wiki.qut.edu.au/pages/viewpage.action?pageId=181178395)
- [Day and Night with Lateral Pose Change Dataset](https://wiki.qut.edu.au/display/cyphy/Day+and+Night+with+Lateral+Pose+Change+Datasets)
- [Challenging data sets for point cloud registration algorithms](https://projects.asl.ethz.ch/datasets/doku.php?id=laserregistration:laserregistration)
- [KITTI Vision Benchmark Suite](http://www.cvlibs.net/datasets/kitti/index.php)
- [Málaga Stereo and Laser Urban Data Set](https://www.mrpt.org/MalagaUrbanDataset)
- [CCSAD (Stereo Urban) Dattaset](http://aplicaciones.cimat.mx/Personal/jbhayet/ccsad-dataset)
- [CityScapes Dataset](https://www.cityscapes-dataset.com/)
- [Rawseeds Outdoor Dataset](http://www.rawseeds.org/home/category/benchmarking-toolkit/datasets/)
- [Oxford Robotcar Dataset](http://robotcar-dataset.robots.ox.ac.uk/)
- [Multi-modal Panoramic 3D Outdoor Dataset (MPO)](http://robotics.ait.kyushu-u.ac.jp/kurazume_lab/research-e.php?content=db)
- [KAIST Day/Night Dataset](https://sites.google.com/view/multispectral/home)
- [Zurich Urban Micro Aerial Vehicle Dataset](http://rpg.ifi.uzh.ch/zurichmavdataset.html)
- [Multi Vehicle Stereo Event Camera Dataset](https://docs.google.com/spreadsheets/d/1mudM7LxXv09ywuQGDp3t_RlIjIdwzg_ZaMu78agLmH4/edit#gid=0)
- [Complex Urban Dataset](http://irap.kaist.ac.kr/dataset/)


### Indoor
- [Cosy Localization Database (COLD)](https://www.pronobis.pro/#data)
- [Challenging data sets for point cloud registration algorithms](https://projects.asl.ethz.ch/datasets/doku.php?id=laserregistration:laserregistration)
- [TUM RGB-D SLAM Dataset and Benchmark](https://vision.in.tum.de/data/datasets/rgbd-dataset)
- [ICL-NUIM RGBD Dataset](https://www.doc.ic.ac.uk/~ahanda/VaFRIC/iclnuim.html)
- [Augmented ICL-NUIM Reconstruction Dataset](http://redwood-data.org/indoor/index.html)
- [Comprehensive RGB-D Benchmark (CoRBS)](http://corbs.dfki.uni-kl.de/?pagerd_tumlltzzf42zsv6de7b9)
- [Event-Camera Dataset and Simulator](http://rpg.ifi.uzh.ch/davis_data.html)
- [SceneNet RBG-D Dataset](https://robotvault.bitbucket.io/scenenet-rgbd.html)


### Terrain
- [Multi-Sensor Perception (Marulan) Dataset ](http://sdi.acfr.usyd.edu.au/)
- [Challenging data sets for point cloud registration algorithms](https://projects.asl.ethz.ch/datasets/doku.php?id=laserregistration:laserregistration)
- [Kagaru Airborne Stereo Dataset Dataset](http://asrl.utias.utoronto.ca/~mdw/kagarudataset.html)
- [Devon Island Rover Navigation Dataset](http://asrl.utias.utoronto.ca/datasets/devon-island-rover-navigation/)
- [Canadian Planetary Emulation Terrain 3D Mapping Dataset](http://asrl.utias.utoronto.ca/datasets/3dmap/#Datasets)
- [Solar-powered UAV Sensing and Mapping Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=fsr2015)
- [Multi-modal Panoramic 3D Outdoor Dataset (MPO)](http://robotics.ait.kyushu-u.ac.jp/kurazume_lab/research-e.php?content=db)
- [Chilean Underground Mine Dataset](http://dataset.amtc.cl/#)
- [Symphony Lake Dataset](http://dream.georgiatech-metz.fr/?q=node/79)
- [Visual-Inertial Canoe Dataset](https://databank.illinois.edu/datasets/IDB-9342111)
- [Sugar Beets 2016, Agricultural Robot Dataset](http://www.ipb.uni-bonn.de/data/sugarbeets2016/)
- [Katwijk Beach Planetary Rover Dataset](https://robotics.estec.esa.int/datasets/katwijk-beach-11-2015/)


### Underwater
- [ACFR Marine Robotics Dataset](http://marine.acfr.usyd.edu.au/datasets/)
- [Underwater Caves SONAR and Vision Dataset](http://cirs.udg.edu/caves-dataset/)
