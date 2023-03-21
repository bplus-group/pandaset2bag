<!-- PROJECT LOGO -->
<br/>
<div align="center">
  <a href="https://www.b-plus.com/de/home">
    <img src="https://www.b-plus.com/fileadmin/data_storage/images/b-plus_Logo.png" alt="Logo" width="150" height="150">
  </a>

  <h3 align="center">pandaset2bag</h3>

  <p align="center">
    Command-line utility (and Python library) for converting the <a href="https://pandaset.org/">PandaSet</a> Dataset to ROS2 bag files
    <br/>
    <a href="#quickstart">Quickstart</a>
    ·
    <a href="https://github.com/bplus-group/pandaset2bag/issues">Report Bug</a>
    ·
    <a href="https://github.com/bplus-group/pandaset2bag/issues">Request Feature</a>
  </p>
</div>
<br/>

<!-- PROJECT SHIELDS -->
<div align="center">

  [![LinkedIn][linkedin-shield]][linkedin-url]
  [![Stars][star-shield]][star-url]

</div>

<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#quickstart">Quickstart</a>
      <ul>
        <li><a href="#installation">Installation</a></li>
        <li><a href="#using-the-command-line-interface">Using the command-line interface</a></li>
        <li><a href="#using-pandaset2bag-in-a-python-script">Using pandaset2bag in a Python script</a></li>
      </ul>
    </li>
    <li><a href="#using-pandaset2bag-with-foxglove">Using pandaset2bag with Foxglove</a></li>
    <li><a href="#coordinate-system">Coordinate system</a></li>
    <li><a href="#annotations">Annotations</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
  </ol>
</details>

---

# pandaset2bag

## Quickstart

### Installation

With the use of the Python library [Rosbags](https://gitlab.com/ternaris/rosbags) which does not have any dependencies on the ROS2 software stacks, *pandaset2bag* can be used without the need to install ROS2. *pandaset2bag* requires an installation of Python 3.8+, as well as pip. Other dependencies can be found in the [requirements.txt](https://github.com/bplus-group/pandaset2bag/blob/master/requirements.txt).

To install from source with pip:

```bash
$ python3 -m pip install git+https://github.com/bplus-group/pandaset2bag
```
<p align="right"><a href="#top">Back to top</a></p>

### Configuration

Per default, *pandaset2bag* uses the `Marker.msg` definition before the [marker textures](https://github.com/ros2/common_interfaces/commit/4b1ba4621b07cf7130997185b25eabcf0e256b59) were added on Aug 2021 to support the visualization of the generated bag files via [Foxglove](https://foxglove.dev/). To use the updated `Marker.msg` definition, you can set the `UPDATED_VISUALIZATION_MSG_MARKER` environment variable to `true`:


```bash
export UPDATED_VISUALIZATION_MSG_MARKER=true
```

</br>When calling the converter, the used `Marker.msg` definition will be displayed at the beginning:

```plaintext
█████ Using DEFAULT/UPDATED visualization_msgs
```

</br>See <a href="#using-pandaset2bag-with-foxglove">below</a> for details on using pandaset2bag with Foxglove.

<p align="right"><a href="#top">Back to top</a></p>

### Using the command-line interface

**basic example**

```bash
$ pandaset2bag --dataset-dir /data/pandaset --sequence-id 2 --output pandasetbag_002
```

```plaintext
█████ Using UPDATED visualization_msgs
Converting cameras... ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ 100% 0:00:06
Converting GPS... ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ 100% 0:00:00
Converting pandar64 lidar... ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ 100% 0:00:01
Converting pandarGT lidar... ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ 100% 0:00:01
Converting cuboids... ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ 100% 0:00:05
Generating /tf for ego_vehicle... ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ 100% 0:00:00
Generating /tf for left_camera... ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ 100% 0:00:00
Generating /tf for front_right_camera... ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ 100% 0:00:00
Generating /tf for front_camera... ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ 100% 0:00:00
Generating /tf for right_camera... ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ 100% 0:00:00
Generating /tf for back_camera... ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ 100% 0:00:00
Generating /tf for front_left_camera... ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ 100% 0:00:00
Generating /tf for pandar64... ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ 100% 0:00:00
Generating /tf for pandarGT... ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ 100% 0:00:00
```
**advanced example**
  - resizing images to 1280x720
  - use JPEG compression for the corresponding topics

```plaintext
$ pandaset2bag --dataset-dir /data/pandaset \
               --sequence-id 2 \
               --max-image-size 1280 \
               --type compressed \
               --output pandasetbag_002
```

#### Options

For more advanced options while converting:

```plaintext
  -h, --help            show this help message and exit
  -d DATASET_DIR, --dataset-dir DATASET_DIR
                        the directory path of the dataset (default: None)
  -i SEQUENCE_ID, --sequence-id SEQUENCE_ID
                        the ID of the sequence to be converted (default: None)
  -o OUTPUT, --output OUTPUT
                        the save path of the rosbag file; if '', i.e the rosbag file will be saved
                        in the current working directory with the name 'pandasetbag_{sequence_id}'
                        (default: )
  -s MAX_IMAGE_SIZE [MAX_IMAGE_SIZE ...], --max-image-size MAX_IMAGE_SIZE [MAX_IMAGE_SIZE ...]
                        maximum image size to convert to (default: [])
  -t {raw,compressed,raw_compressed}, --type {raw,compressed,raw_compressed}
                        type to be used to convert an image (default: raw)
  -f {jpeg,png}, --format {jpeg,png}
                        image format used for compression (default: jpeg)
  -q JPEG_QUALITY, --jpeg-quality JPEG_QUALITY
                        image compression quality for JPEG format; the image quality, on a scale
                        from 0 (worst) to 95 (best), or the string keep. Values above 95 should be
                        avoided; 100 disables portions of the JPEG compression algorithm, and
                        results in large files with hardly any gain in image quality; the value keep
                        is only valid for JPEG files and will retain the original image quality
                        level, subsampling, and qtables (default: 75)
  -l {0,1,2,3,4,5,6,7,8,9}, --png-compress-level {0,1,2,3,4,5,6,7,8,9}
                        image compression level for PNG format; ZLIB compression level, a number
                        between 0 and 9: 1 gives best speed, 9 gives best compression, 0 gives no
                        compression at all. When optimize option is True compress_level has no
                        effect (it is set to 9 regardless of a value passed); the value is only
                        valid for PNG files and will be otherwise ignored (default: 6)
  -O, --png-optimize    image optimization status for PNG format; if True, instructs the PNG writer
                        to make the output file as small as possible; this includes extra processing
                        in order to find optimal encoder settings; the value is only valid for PNG
                        files and will be otherwise ignored (default: False)
  -m {none,file,message}, --mode {none,file,message}
                        compression mode for rosbag file. (default: none)
  -c, --cuboids         save cuboids `DataFrame` status; if True, save the converted cuboids
                        `DataFrame`'s as `.pkl.gz` files, representing the properties of the cuboids
                        in normalized ego coordinates (default: False)
  -v, --version         show program's version number and exit
```
<p align="right"><a href="#top">Back to top</a></p>

### Using pandaset2bag in a Python script

**basic example**

```python
from pandaset2bag.pandaset2bag_converter import PandaSet2BagConverter

converter = PandaSet2BagConverter('./data/pandaset')

sequence_id = '002'
converter.convert(sequence_id)
```

**advanced example**
- resizing images to 1280x720
- convert the image both as `Image.msg` and as `CompressedImage.msg`
- use JPEG compression for the corresponding topics
- enable compression (equivalent to `ros2 bag ... --compression-mode file`)

```python
from pandaset2bag.pandaset2bag_converter import PandaSet2BagConverter
from pandaset2bag.enums import CompressedImageFormat, ImageConvertType
from rosbags.rosbag2 import Writer

converter = PandaSet2BagConverter('./data/pandaset')

converter.image_convert_type = ImageConvertType.RAW_COMPRESSED
converter.image_format = CompressedImageFormat.JPEG
converter.max_image_size = (1280, 720)
converter.compression_mode = Writer.CompressionMode.FILE

sequence_id = '002'
converter.convert(sequence_id)
```
<p align="right"><a href="#top">Back to top</a></p>

## Using *pandaset2bag* with Foxglove

As mentioned <a href="#configuration">before</a>, to visualize the generated bag files (`.db3`; sqlite3 storage plugin) with Foxglove, use the `Marker.msg` definition set as default.

> **Warning**
> Foxglove does not support the playback of compressed (.db3) bag files.

For increased performance during playback or to support the compression of bag files, the use of [MCAP](https://mcap.dev/) is recommended. For conversion from `.db3` to `.mcap`, the corresponding [MCAP CLI](https://github.com/foxglove/mcap/tree/main/go/cli/mcap) provided by Foxglove can be used.

> **Warning**
> MCAP CLI uses the updated `Marker.msg` definition so you have to set the <a href="#configuration">`UPDATED_VISUALIZATION_MSG_MARKER`</a> environment variable to `true` while converting a sequence from the PandaSet using *pandaset2bag*.

<p align="right"><a href="#top">Back to top</a></p>

## Coordinate system

It is important to note that in the PandaSet dataset, all point cloud data is referenced to a global coordinate system and not an ego coordinate system. See [arXiv:2112.12610](https://arxiv.org/abs/2112.12610) for more details.

Using *pandaset2bag* all data will refer to an ego coordinate system where the coordinates are transformed to a unified normative coordinate system, such that the x-axis corresponds positive to the front direction, the y-axis corresponds positive to the left direction, and the z-axis corresponds positive to the top direction.

<p align="right"><a href="#top">Back to top</a></p>

## Annotations

### Cuboids

The LiDAR cuboid annotations can be accessed as JSON string through the `text` property of a `Marker.msg` from the `/panda/markers` topic of each scene/sequence. Using [`pandas.DataFrame.to_json`](https://pandas.pydata.org/docs/reference/api/pandas.DataFrame.to_json.html) with `orient='table'`, the string also includes the JSON schema.

Additionally there is the option to export the LiDAR cuboid annotations as [`pandas.DataFrame`](https://pandas.pydata.org/docs/reference/api/pandas.DataFrame.html) (see <a href="#options">options</a> using the *pandaset2bag* CLI as an example). Using [`pandas.DataFrame.to_pickle`](https://pandas.pydata.org/docs/reference/api/pandas.DataFrame.to_pickle.html) with `compression='gzip'` the exported cuboid annotations are fully compatible with the [pandaset-devkit](https://github.com/scaleapi/pandaset-devkit).

<p align="right"><a href="#top">Back to top</a></p>

### Semantic segmentation

Similary the semantic segmentation annotations can be accessed through the binary data blob of a [`PointCloud2.msg`](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html) from the corresponding LiDAR topic (e.g. `/panda/ego_vehicle/pandar64`) of each scene. Each `PointCloud2.msg` has an additional channel ([`PointField.msg`](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointField.html)) for the class ID of the semantic segmentation result. The channels of a `PointCloud2.msg` are `x`, `y`, `z`, `intensity` and `class_id` each of datatype FLOAT32.

> **Note**
> Not all scenes of the PandaSet have semantic segmentation annotations. In that case, the class ID is set to -1.

</br>Using Foxglove you can color the point cloud by the `class_id` using the *'Color by'* option under the correspondig LiDAR topic.

<p align="right"><a href="#top">Back to top</a></p>

## Contributing

If you have a suggestion that would improve this, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!


1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/NewFeature`)
3. Commit your Changes (`git commit -m 'Add some NewFeature'`)
4. Push to the Branch (`git push origin feature/NewFeature`)
5. Open a Pull Request

<p align="right"><a href="#top">Back to top</a></p>

## License

All code, unless otherwise noted, is licensed under the MIT License. See [`LICENSE`](https://github.com/bplus-group/pandaset2bag/blob/master/LICENSE) for more information.

<p align="right"><a href="#top">Back to top</a></p>


<!---Links And Images -->
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&color=808080
[linkedin-url]: https://de.linkedin.com/company/b-plus-group
[star-shield]: https://img.shields.io/github/stars/bplus-group/pandaset2bag.svg?style=for-the-badge&color=144E73&labelColor=808080
[star-url]: https://github.com/bplus-group/pandaset2bag
