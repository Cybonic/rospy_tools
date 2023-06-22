

GPS_TOPIC="/vectornav/GPS_INS"
TARGET_DIR="/dream-storage/Backpack_Dataset/cross-seasonal-semantic-dataset/lake-bPearl/semantic_bags/on-foot"
DST_FLDR="/home/GPU/tbarros"
echo "TARGET DIR: ${TARGET_DIR}"
echo "GPS_TOPIC: ${GPS_TOPIC}"
echo "===================================="

python3 merge_bag_to_kmz.py  --target_bag_dir "${TARGET_DIR}" --gps "${GPS_TOPIC}" --dst_root "${DST_FLDR}"