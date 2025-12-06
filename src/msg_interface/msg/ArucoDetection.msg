# 인식된 마커의 ID
int32 marker_id

# 마커의 절대 월드 좌표 (운반 로봇에게 전달할 위치)
float64 world_x
float64 world_y
float64 world_z

# 마커가 카메라 기준으로 얼마나 떨어져 있는지 (선택 사항이지만 유용함)
float64 distance_to_marker