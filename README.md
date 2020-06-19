# MATLAB READ ME
Open the script up in Matlab and change the src and dst variables in the USER SETUP area. The src variable should contain the absolute path to the folder containing the rosbags you need to extract images from. The dst variable should specify the absolute path to where you want the images extracted to. One folder for each image topic will be created within the destination folder.


# PYTHON READ ME
## Setup

To install the required dependencies run the following command:

```
pip install -r requirements.txt
```

## Running the script

To run the script you simply need to provide a source folder containing bag files and destination directory to save images to.

For example:

```
./rosbag_extract bags/ images/
```

The script will create a folder for each topic inside images and save image messages for each topic into their respective folders.
