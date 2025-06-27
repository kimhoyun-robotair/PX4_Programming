import math

EARTH_RADIUS = 6371000.0  # 지구 반경 (미터)

class GeoLibrary:
    def __init__(self):
        self.ref_init_done = False

    def init_reference(self, lat_0, lon_0, timestamp):
        """
        기준점 위도/경도와 타임스탬프를 설정하여, 나중에 투영에 사용할 내부 값을 초기화합니다.
        """
        self.ref_timestamp = timestamp
        self.ref_lat = math.radians(lat_0)
        self.ref_lon = math.radians(lon_0)
        self.ref_sin_lat = math.sin(self.ref_lat)
        self.ref_cos_lat = math.cos(self.ref_lat)
        self.ref_init_done = True

    def project(self, lat, lon):
        """
        주어진 위도와 경도(degrees)를 기준점에 대해 Azimuthal Equidistant Projection 방식으로 투영하여
        로컬 좌표 (x, y) 를 미터 단위로 계산합니다.
        """
        # 입력 좌표를 라디안으로 변환
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        sin_lat = math.sin(lat_rad)
        cos_lat = math.cos(lat_rad)
        cos_d_lon = math.cos(lon_rad - self.ref_lon)

        # 내적 값 계산 후 -1~1 범위로 제한
        arg = self.ref_sin_lat * sin_lat + self.ref_cos_lat * cos_lat * cos_d_lon
        arg = max(min(arg, 1.0), -1.0)
        c = math.acos(arg)

        # c가 0이 아니면 비율 k 계산 (c/sin(c))
        k = c / math.sin(c) if abs(c) > 1e-6 else 1.0

        x = k * (self.ref_cos_lat * sin_lat - self.ref_sin_lat * cos_lat * cos_d_lon) * EARTH_RADIUS
        y = k * cos_lat * math.sin(lon_rad - self.ref_lon) * EARTH_RADIUS

        return x, y
    def is_waypoint_reached():
        pass


# 사용 예시:
if __name__ == '__main__':
    mp = GeoLibrary()
    # 기준점을 예: 위도 36.661078, 경도 126.342193, 타임스탬프는 현재 시각
    mp.init_reference(37.449187,126.653021,0.0)
    x, y = mp.project(37.449187,126.653021)
    print(f"Projected local coordinates: x={x:.2f} m, y={y:.2f} m")
    x, y = mp.project(37.452717,126.653195)
    print(f"Projected local coordinates: x={x:.2f} m, y={y:.2f} m")
    x, y = mp.project(37.454559,126.657573)
    print(f"Projected local coordinates: x={x:.2f} m, y={y:.2f} m")
    x, y = mp.project(37.454490,126.648858)
    print(f"Projected local coordinates: x={x:.2f} m, y={y:.2f} m")
    x, y = mp.project(37.452717,126.653195)
    print(f"Projected local coordinates: x={x:.2f} m, y={y:.2f} m")
    x, y = mp.project(37.452059,126.647888)
    print(f"Projected local coordinates: x={x:.2f} m, y={y:.2f} m")
    x, y = mp.project(37.451786,126.659597)
    print(f"Projected local coordinates: x={x:.2f} m, y={y:.2f} m")
    x, y = mp.project(37.452717,126.653195)
    print(f"Projected local coordinates: x={x:.2f} m, y={y:.2f} m")
    x, y = mp.project(37.449187,126.653021)
    print(f"Projected local coordinates: x={x:.2f} m, y={y:.2f} m")
