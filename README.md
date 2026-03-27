# ⚡ 코드 리뷰 포트폴리오 
핵심문제해결사례(상세 기술 문서)에 기술된 내용과 매핑되는 코드 리뷰용 포트폴리오입니다.

# 📌 목록

## 1. NavMesh Pathfinding 최적화 및 코너 주행 보정
**NavMesh Pathfinding Optimization & Corner Navigation Fix**

> DirectX 11 퍼니싱: 그레이 레이븐 프로젝트 中

NavMesh 셀 중심 기반 A* 경로의 지그재그 문제를 SSFA로 압축하고,  
코너 지점 이동 불가 문제를 외적 기반 Waypoint Nudge로 해결한 구현입니다.

# 파일 구성

| 파일 | 설명 |
|---|---|
| [`Navigation.h`](./NavMeshPathFindingOpt/Navigation.h) | 클래스 인터페이스, 멤버 변수, 자료구조 정의 |
| [`Navigation.cpp`](./NavMeshPathFindingOpt/Navigation.cpp) | 길찾기 전체 구현 (A*, SSFA, Corner Nudge) |

---

# 읽는 순서
2. **`Navigation.h`** — `Build_PathTo()` public API 및 내부 자료구조 확인
3. **`Navigation.cpp`** — 아래 흐름대로 핵심 함수 탐색

```
Build_PathTo()                             ← 진입점. 전체 흐름 확인
  ├─ Build_CellPath()                      ← A* 탐색
  ├─ Build_Waypoints_SimpleStupidFunnel()  ← 경로 압축 (SSFA)
  │    ├─ Build_PortalList()               ← Portal 구성
  │    └─ Run_Funnel()                     ← Funnel 실행
  └─ Nudge_Waypoints_AlongPath()           ← 코너 보정
```

> 각 파일은 엔진 컨텍스트를 전제로 작성되어 있어 독립 빌드보다는 로직과 설계 의도 파악 목적으로 열람하는 것을 권장합니다.  
