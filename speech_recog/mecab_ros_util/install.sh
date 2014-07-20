# START_DIR=`pwd` ;
# MECAB_ROS_ROOT=`rospack find mecab_ros_util`/3rdparty ;
# TARGET_MECAB_PATH=mecab-python-0.996 ;
# MECAB_ROOT_PATH=mecab ;
# cd $MECAB_ROS_ROOT ;
# wget https://mecab.googlecode.com/files/$TARGET_MECAB_PATH.tar.gz ;
# tar zxvf $TARGET_MECAB_PATH.tar.gz ;
# mv $TARGET_MECAB_PATH $MECAB_ROOT_PATH ;
# cd $START_DIR ;

sudo apt-get install libmecab-dev libmecab1 mecab mecab-jumandic mecab-utils mecab-ipadic mecab-ipadic-utf8 python-mecab ipadic ;
## please change configuration of pythonpath correctly.
