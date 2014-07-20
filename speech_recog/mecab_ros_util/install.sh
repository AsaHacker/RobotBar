START_DIR=`pwd` ;
MECAB_ROS_ROOT=`rospack find mecab_ros_util`/3rdparty ;
TARGET_MECAB_PATH=mecab-python-0.996.tar.gz ;
MECAB_ROOT_PATH=mecab ;
cd $MECAB_ROS_ROOT ;
wget https://mecab.googlecode.com/files/$TARGET_MECAB_PATH ;
tar zxvf $TARGET_MECAB_PATH ;
mv $TARGET_MECAB_PATH $MECAB_ROOT_PATH ;
cd $START_DIR ;

