#include "Navigation.h"
#include "Cell.h"
#include "GameInstance.h"
#include <queue>

_float4x4 CNavigation::m_WorldMatrix = {};

CNavigation::CNavigation(ID3D11Device* _pDevice, ID3D11DeviceContext* _pContext)
    : CComponent(_pDevice, _pContext)
{
    XMStoreFloat4x4(&m_WorldMatrix, XMMatrixIdentity());
}

CNavigation::CNavigation(const CNavigation& _Prototype)
    : CComponent(_Prototype)
    , m_Cells{ _Prototype.m_Cells }
    , m_iCellCount(_Prototype.m_iCellCount)
#ifdef _DEBUG
    , m_pShader{ _Prototype.m_pShader }
#endif
{
#ifdef _DEBUG
    Safe_AddRef(m_pShader);
    m_pInputLayout = _Prototype.m_pInputLayout;
    m_pBatch = _Prototype.m_pBatch;
    m_pEffect = _Prototype.m_pEffect;
    Safe_AddRef(m_pInputLayout);
#endif
    // Cell 정보는 얕은 복사
    for (auto& pCell : m_Cells)
        Safe_AddRef(pCell);
}

HRESULT CNavigation::Initialize_Prototype(const _tchar* _pNavigationDataFile)
{
    // 미리 저장된 NavMesh 데이터 파일을 읽어 원본 Cell 목록을 구성한다
    _ulong  dwByte = {};
    HANDLE  hFile = CreateFile(_pNavigationDataFile, GENERIC_READ, 0, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
    if (INVALID_HANDLE_VALUE == hFile)
        return E_FAIL;

    _uint iSize = 0;
    ReadFile(hFile, &iSize, sizeof(_uint), &dwByte, nullptr);

    while (true)
    {
        _float3 vPoints[3] = {};
        _uint   iOption = 0;
        ReadFile(hFile, vPoints, sizeof(_float3) * 3, &dwByte, nullptr);
        ReadFile(hFile, &iOption, sizeof(_uint), &dwByte, nullptr);

        if (0 == dwByte) break; // 읽을 데이터 없으면 탈출

        CCell* pCell = CCell::Create(m_pDevice, m_pContext, vPoints, m_Cells.size(), iOption);
        if (nullptr == pCell)
            return E_FAIL;

        m_Cells.push_back(pCell);
    }
    CloseHandle(hFile);

    if (!m_Cells.empty())
        m_iCellCount = m_Cells.size();

    // 모든 Cell 로드 후 인접 Cell 연결
    if (FAILED(SetUp_Neighbor()))
        return E_FAIL;

#ifdef _DEBUG
    m_pShader = CShader::Create(m_pDevice, m_pContext, TEXT("../../EngineSDK/hlsl/Shader_Cell.hlsl"), VTXPOS::Elements, VTXPOS::iNumElements);
    if (nullptr == m_pShader)
        return E_FAIL;

    m_pBatch = new PrimitiveBatch<VertexPositionColor>(m_pContext);
    m_pEffect = new BasicEffect(m_pDevice);

    const void* pShaderByteCode = nullptr;
    size_t      iShaderByteCodeLength = 0;
    m_pEffect->SetVertexColorEnabled(true);
    m_pEffect->GetVertexShaderBytecode(&pShaderByteCode, &iShaderByteCodeLength);

    if (FAILED(m_pDevice->CreateInputLayout(VertexPositionColor::InputElements, VertexPositionColor::InputElementCount, pShaderByteCode, iShaderByteCodeLength, &m_pInputLayout)))
        return E_FAIL;
#endif

    return S_OK;
}

HRESULT CNavigation::Initialize(void* _pArg)
{
    if (nullptr == _pArg)
        return S_OK;

    NAVIGATION_DESC* pDesc = static_cast<NAVIGATION_DESC*>(_pArg);
    m_iCurCellIndex = pDesc->iStartIndex;

    m_gCost.resize(m_iCellCount, 0.0f);
    m_Parents.resize(m_iCellCount, -1);
    m_ParentExitLines.resize(m_iCellCount, -1);
    m_Stamps.resize(m_iCellCount, 0);

    return S_OK;
}

void CNavigation::Update(_fmatrix _vMapWorldmatrix)
{
    XMStoreFloat4x4(&m_WorldMatrix, _vMapWorldmatrix);
}

void CNavigation::Setting_CurCellIndex(_fvector _vOwnerWorldPos)
{
    m_iCurCellIndex = Find_CellIndex(_vOwnerWorldPos);
    assert(m_iCurCellIndex != -1);
}

int CNavigation::Find_CellIndex(_fvector _vTargetPos)
{
    const _matrix matMapWorld = XMLoadFloat4x4(&m_WorldMatrix);
    const _matrix matWorldToMap = XMMatrixInverse(nullptr, matMapWorld);

    _vector vMapLocalPos = XMVector3TransformCoord(_vTargetPos, matWorldToMap);
    for (size_t i = 0; i < m_Cells.size(); ++i)
    {
        if (true == m_Cells[i]->Search_StartCell(vMapLocalPos))
            return i;
    }
    return -1;
}

_bool CNavigation::Is_Move(_fvector _vOwnerWorldPos, _float3* _pOutLineNormal)
{
    // World -> Map 로컬 좌표계로 변환
    const _matrix matMapWorld = XMLoadFloat4x4(&m_WorldMatrix);
    const _matrix matWorldToMap = XMMatrixInverse(nullptr, matMapWorld);
    _vector vMapLocalPos = XMVector3TransformCoord(_vOwnerWorldPos, matWorldToMap);

    _int iNeighborIndex = -1;

    if (true == m_Cells[m_iCurCellIndex]->Is_In(vMapLocalPos, iNeighborIndex, _pOutLineNormal))
    {
        return true;
    }
    else
    {
        if (-1 == iNeighborIndex)
            return false;

        while (true)
        {
            // 루프 중 이웃이 -1 → 실질적으로 이동 불가인 경로
            if (-1 == iNeighborIndex)
                return false;

            if (true == m_Cells[iNeighborIndex]->Is_In(vMapLocalPos, iNeighborIndex, _pOutLineNormal))
            {
                m_iCurCellIndex = iNeighborIndex;
                return true;
            }
        }
    }
}

_vector CNavigation::Adjust_CellHeight(_fvector _vPos)
{
    const _matrix matMapWorld = XMLoadFloat4x4(&m_WorldMatrix);
    const _matrix matWorldToMap = XMMatrixInverse(nullptr, matMapWorld);

    _vector vMapLocalPos = XMVector3TransformCoord(_vPos, matWorldToMap);
    vMapLocalPos = XMVectorSetY(vMapLocalPos, Compute_Height(m_iCurCellIndex, vMapLocalPos));

    return XMVector3TransformCoord(vMapLocalPos, XMLoadFloat4x4(&m_WorldMatrix));
}

HRESULT CNavigation::SetUp_Neighbor()
{
    for (auto& pSourCell : m_Cells)
    {
        for (auto& pDestCell : m_Cells)
        {
            if (pSourCell == pDestCell)
                continue;

            // 공유 정점 쌍으로 인접 여부 판별 후 이웃 인덱스 등록
            if (true == pDestCell->Compare_Points(pSourCell->Get_Point(CCell::POINT_A), pSourCell->Get_Point(CCell::POINT_B)))
                pSourCell->Set_NeighborIndex(CCell::LINE_AB, pDestCell);
            else if (true == pDestCell->Compare_Points(pSourCell->Get_Point(CCell::POINT_B), pSourCell->Get_Point(CCell::POINT_C)))
                pSourCell->Set_NeighborIndex(CCell::LINE_BC, pDestCell);
            else if (true == pDestCell->Compare_Points(pSourCell->Get_Point(CCell::POINT_C), pSourCell->Get_Point(CCell::POINT_A)))
                pSourCell->Set_NeighborIndex(CCell::LINE_CA, pDestCell);
        }
    }
    return S_OK;
}

_float CNavigation::Compute_Height(_int _iCurCellIndex, _fvector _vPos)
{
    if (-1 == _iCurCellIndex)
        return 0.0f;

    return m_Cells[_iCurCellIndex]->Compute_Height(_vPos);
}

// ------------------------------------------------------------
// Build_PathTo
// 외부에 공개하는 길찾기 진입점
// 1) A* → 2) SSFA(경로 압축) → 3) Corner Nudge → 결과 반환
// SSFA 실패 시 Cell 중점 기반 경로로 Fallback
// ------------------------------------------------------------
_bool CNavigation::Build_PathTo(const _fvector _vStartPos, const _fvector _vTargetPos, vector<_float3>& _OutWaypoints)
{
    // 1. Path Finding (A*)
    if (!Build_CellPath(_vStartPos, _vTargetPos))
        return false;

    // 2. Simple Stupid Funnel
    vector<_float3> NewWaypoints;
    NewWaypoints.reserve(m_CellPath.size());

    if (!Build_Waypoints_SimpleStupidFunnel(_vStartPos, _vTargetPos, NewWaypoints))
    {
        // Fallback: Cell Centroid
        if (!Build_Waypoints_CellCentroid(_vStartPos, _vTargetPos, NewWaypoints))
            return false;
    }
    else
    {
        // 3. Waypoints 코너 보정
        Nudge_Waypoints_AlongPath(NewWaypoints);
    }

    // 4. 결과 반환
    _OutWaypoints.swap(NewWaypoints);
    m_Waypoints = _OutWaypoints;
    return true;
}

// ------------------------------------------------------------
// Build_CellPath
// A* 알고리즘으로 Cell 인덱스 경로를 탐색한다
// H = 목적지까지의 직선 거리
// G = 시작점부터 현재까지의 누적 비용
// F = G + H
// ------------------------------------------------------------
_bool CNavigation::Build_CellPath(const _fvector _vStartPos, const _fvector _vTargetPos)
{
    if (m_Cells.empty())
        return false;

    const _int iStartIndex = Find_CellIndex(_vStartPos);
    const _int iTargetIndex = Find_CellIndex(_vTargetPos);

    if (iStartIndex < 0 || iStartIndex >= m_iCellCount ||
        iTargetIndex < 0 || iTargetIndex >= m_iCellCount)
        return false;

    if (iStartIndex == iTargetIndex)
    {
        m_CellPath.clear();
        m_PortalLines.clear();
        m_CellPath.push_back(iStartIndex);
        return true;
    }

    // Stamp 오버플로우 방지
    if (++m_iCurStamp == UINT_MAX)
    {
        std::fill(m_Stamps.begin(), m_Stamps.end(), 0);
        m_iCurStamp = 1;
    }

    m_Stamps[iStartIndex] = m_iCurStamp;
    m_gCost[iStartIndex] = 0.0f;
    m_Parents[iStartIndex] = -1;
    m_ParentExitLines[iStartIndex] = -1;

    priority_queue<AStarNode, vector<AStarNode>, MinF> OpenList;
    _float fStart_H = Compute_Heuristic(iStartIndex, iTargetIndex);
    OpenList.push({ iStartIndex, fStart_H, 0.0f });

    AStarNode tCurNode{};
    _bool     bFound = false;

    while (!OpenList.empty())
    {
        tCurNode = OpenList.top(); OpenList.pop();
        _int iCurIndex = tCurNode.index;

        if (m_Stamps[iCurIndex] != m_iCurStamp) continue; // 만료된 노드
        if (tCurNode.G > m_gCost[iCurIndex])    continue; // Stale 노드

        if (iCurIndex == iTargetIndex) { bFound = true; break; }

        _vector     vCurCenter = m_Cells[iCurIndex]->Get_Center_Simd();
        const _int* pNeighborIndices = m_Cells[iCurIndex]->Get_Neighbors();

        for (int i = 0; i < CCell::LINE_LAST; ++i)
        {
            _int iNextIndex = pNeighborIndices[i];
            if (iNextIndex == -1) continue;

            // Lazy Init
            if (m_Stamps[iNextIndex] != m_iCurStamp)
            {
                m_Stamps[iNextIndex] = m_iCurStamp;
                m_gCost[iNextIndex] = FLT_MAX;
                m_Parents[iNextIndex] = -1;
                m_ParentExitLines[iNextIndex] = -1;
            }

            _vector vNextCenter = m_Cells[iNextIndex]->Get_Center_Simd();
            _float  fDist = Dist3(vNextCenter, vCurCenter);
            _float  fNext_G = m_gCost[iCurIndex] + fDist;

            if (fNext_G < m_gCost[iNextIndex])
            {
                m_gCost[iNextIndex] = fNext_G;
                m_Parents[iNextIndex] = iCurIndex;
                m_ParentExitLines[iNextIndex] = i; // Parent -> Child 방향 ExitLine (Parent 기준), 캐싱해 Left/Right 별도 판단 제거
                _float fNext_H = Compute_Heuristic(iNextIndex, iTargetIndex);
                OpenList.push({ iNextIndex, fNext_G + fNext_H, fNext_G });
            }
        }
    }

    if (!bFound)
        return false;

    // 역방향 경로 복원
    vector<_int> ReversedCellPath;   ReversedCellPath.reserve(64);
    vector<_int> ReversedPortalLines; ReversedPortalLines.reserve(64);
    _int v = iTargetIndex;
    _int guard = 0;

    while (v != -1)
    {
        ReversedCellPath.push_back(v);
        ReversedPortalLines.push_back(m_ParentExitLines[v]); // index 0은 dummy(-1)
        if (v == iStartIndex) break;

        v = m_Parents[v];
        if (++guard > m_iCellCount) return false;
    }

    if (ReversedCellPath.empty() || ReversedCellPath.back() != iStartIndex)
        return false;

    m_CellPath.assign(ReversedCellPath.rbegin(), ReversedCellPath.rend());
    m_PortalLines.assign(ReversedPortalLines.rbegin(), ReversedPortalLines.rend());

#ifdef _DEBUG
    for (auto& cell : m_Cells)
        cell->ChangeRenderColor({ 1.0f, 1.0f, 1.0f, 1.0f });

    for (int& idx : m_CellPath)
    {
        m_Cells[idx]->ChangeRenderColor({ 1.0f, 0.0f, 0.0f, 1.0f });
        if (idx == iTargetIndex) m_Cells[idx]->ChangeRenderColor({ 0.0f, 1.0f, 0.0f, 1.0f });
        else if (idx == iStartIndex)  m_Cells[idx]->ChangeRenderColor({ 0.0f, 0.0f, 1.0f, 1.0f });
    }
#endif // test code

    assert(m_PortalLines.size() == m_CellPath.size());
    if (m_CellPath.size() >= 2) assert(m_PortalLines[0] == -1);
    else                        assert(m_PortalLines[0] == -1);

    return true;
}

_bool CNavigation::Build_Waypoints_CellCentroid(const _fvector _vStartPos, const _fvector _vTargetPos, vector<_float3>& _OutWaypoints)
{
    if (m_CellPath.empty())
        return false;

    const _int iSize = (_int)m_CellPath.size();
    _OutWaypoints.clear();
    _OutWaypoints.reserve(iSize);

    _float3 vStartPos{}, vTargetPos{};
    XMStoreFloat3(&vStartPos, _vStartPos);
    XMStoreFloat3(&vTargetPos, _vTargetPos);

    if (iSize < 2)
    {
        _OutWaypoints.push_back(vStartPos);
        _OutWaypoints.push_back(vTargetPos);
        return true;
    }

    // 중간 Cell들의 중점을 Waypoint로 사용
    _OutWaypoints.push_back(vStartPos);
    for (_int i = 1; i < iSize - 1; ++i)
        _OutWaypoints.push_back(m_Cells[m_CellPath[i]]->Get_Center());
    _OutWaypoints.push_back(vTargetPos);

    return true;
}

_bool CNavigation::Build_Waypoints_SimpleStupidFunnel(const _fvector _vStartPos, const _fvector _vTargetPos, vector<_float3>& _OutWaypoints)
{
    if (m_CellPath.empty())
        return false;

    const _int iSize = (_int)m_CellPath.size();
    _OutWaypoints.clear();
    _OutWaypoints.reserve(iSize);

    if (iSize < 2)
    {
        _float3 vStartPos{}, vTargetPos{};
        XMStoreFloat3(&vStartPos, _vStartPos);
        XMStoreFloat3(&vTargetPos, _vTargetPos);
        _OutWaypoints.push_back(vStartPos);
        _OutWaypoints.push_back(vTargetPos);
        return true;
    }

    if (m_PortalLines.size() != m_CellPath.size())
        return false;

    // 1. Portal List 구성 → 2. Funnel 실행
    if (!Build_PortalList())    return false;
    if (!Run_Funnel(_vStartPos, _vTargetPos, _OutWaypoints)) return false;

    return true;
}

_bool CNavigation::Build_PortalList()
{
    const _int iSize = (_int)m_CellPath.size();
    if (iSize < 2) return false;

    m_Portals.clear();
    m_Portals.reserve(iSize - 1);

    for (_int i = 1; i < iSize; ++i)
    {
        const _int iChild = m_CellPath[i];
        const _int iParent = m_CellPath[i - 1];
        const _int iLineInParent = m_PortalLines[i];

        if (iChild < 0 || iChild >= m_iCellCount)     return false;
        if (iParent < 0 || iParent >= m_iCellCount)     return false;
        if (iLineInParent < 0 || iLineInParent >= CCell::LINE_LAST) return false;

        // Cell Line(CW) 기준: a = left, b = right (parent 진행 방향 기준 정렬)
        LineSegment3 tLine = m_Cells[iParent]->Get_Line(CCell::LINE(iLineInParent));
        m_Portals.push_back({ tLine.a, tLine.b });
    }

    return true;
}

// ------------------------------------------------------------
// Run_Funnel  — Simple Stupid Funnel Algorithm
//
// [좌표계 전제]
//   - Left-Handed, Y-Up
//   - Funnel은 XZ 평면에서 수행 (Y=0 투영)
//   - m_Portals[i].left/right 는 parent→child 진행 방향 기준 정렬
//
// [TriArea2 부호 규약]  (Left-Handed, Y-Up, XZ 평면)
//   TriArea2(a,b,c) = Cross(b-a, c-a).y
//   - 양수: c가 (a→b) 기준 왼쪽
//   - 음수: c가 (a→b) 기준 오른쪽
//   - 0   : collinear
//
// [Tighten 조건]
//   LEFT  — portalLeft가 left 레이의 오른쪽(음수)으로 들어오면 조임 시도
//     TriArea2(apex, left,  portalLeft) >= -eps
//     조임 실패 시 (right 레이 바깥, 양수): Right를 코너로 확정
//     TriArea2(apex, right, portalLeft) <= eps  →  바깥
//
//   RIGHT — portalRight가 right 레이의 왼쪽(양수)으로 들어오면 조임 시도
//     TriArea2(apex, right, portalRight) <= eps
//     조임 실패 시 (left 레이 바깥, 음수): Left를 코너로 확정
//     TriArea2(apex, left,  portalRight) >= -eps  →  바깥
// ------------------------------------------------------------
_bool CNavigation::Run_Funnel(const _fvector _vStartPos, const _fvector _vTargetPos, vector<_float3>& _OutWaypoints)
{
    _OutWaypoints.clear();

    const int iPathCellCount = (int)m_CellPath.size();
    if (iPathCellCount < 1) return false;

    const int iPortalCount = (int)m_Portals.size(); // N - 1
    if (iPathCellCount >= 2 && iPortalCount != iPathCellCount - 1)
        return false;

    _float3 vStartPos{}, vTargetPos{};
    XMStoreFloat3(&vStartPos, _vStartPos);
    XMStoreFloat3(&vTargetPos, _vTargetPos);

    if (iPathCellCount < 2)
    {
        _OutWaypoints.push_back(vStartPos);
        _OutWaypoints.push_back(vTargetPos);
        return true;
    }

    constexpr float kAreaEps = 1e-6f; // collinear/수치 떨림 안정화
    constexpr float kPointEps = 1e-4f;
    constexpr float kPointEpsSq = kPointEps * kPointEps;

    auto ProjectXZ = [](const _float3& v) -> _vector
        { return XMVectorSetY(XMLoadFloat3(&v), 0.f); };

    auto IsSamePointXZ = [&](FXMVECTOR a, FXMVECTOR b) -> _bool
        {
            const _vector d = XMVectorSetY(XMVectorSubtract(a, b), 0.f);
            return XMVectorGetX(XMVector3LengthSq(d)) <= kPointEpsSq;
        };

    auto PushPoint = [&](FXMVECTOR v) -> void
        {
            _float3 vPoint{}; XMStoreFloat3(&vPoint, v);
            if (_OutWaypoints.empty()) { _OutWaypoints.push_back(vPoint); return; }
            if (IsSamePointXZ(v, XMLoadFloat3(&_OutWaypoints.back()))) return; // 중복 제거
            _OutWaypoints.push_back(vPoint);
        };

    // 시작점과 끝점을 폭 0인 sentinel 포탈로 추가해 일반 포탈과 동일하게 처리
    vector<NavPortal> Portals;
    Portals.reserve(iPortalCount + 2);
    Portals.push_back({ vStartPos, vStartPos });
    for (int i = 0; i < iPortalCount; ++i)
        Portals.push_back({ m_Portals[i].left, m_Portals[i].right });
    Portals.push_back({ vTargetPos, vTargetPos });

    const int n = (int)Portals.size(); // iPortalCount + 2

    _vector vApex = ProjectXZ(Portals[0].left);
    _vector vLeft = ProjectXZ(Portals[0].left);
    _vector vRight = ProjectXZ(Portals[0].left);
    int iApexIndex = 0;
    int iLeftIndex = 0;
    int iRightIndex = 0;

    PushPoint(vApex); // 시작점 추가

    for (int i = 1; i < n; )
    {
        const _vector vPortalLeft = ProjectXZ(Portals[i].left);
        const _vector vPortalRight = ProjectXZ(Portals[i].right);

        // 1. Left 레이 업데이트
        {
            const float fArea = TriArea2(vApex, vLeft, vPortalLeft);
            if (fArea >= -kAreaEps)
            {
                const _bool isLeftAtApex = IsSamePointXZ(vLeft, vApex);
                const _bool isInside = (TriArea2(vApex, vRight, vPortalLeft) <= kAreaEps);

                if (isLeftAtApex || isInside)
                {
                    vLeft = vPortalLeft;
                    iLeftIndex = i;
                }
                else
                {
                    // Right를 코너로 확정
                    PushPoint(vRight);
                    vApex = vRight;
                    iApexIndex = iRightIndex;

                    vLeft = vRight = vApex;
                    iLeftIndex = iRightIndex = iApexIndex;

                    i = iApexIndex;
                    continue;
                }
            }
        }

        // 2. Right 레이 업데이트
        {
            const float fArea = TriArea2(vApex, vRight, vPortalRight);
            if (fArea <= kAreaEps)
            {
                const _bool isRightAtApex = IsSamePointXZ(vRight, vApex);
                const _bool isInside = (TriArea2(vApex, vLeft, vPortalRight) >= -kAreaEps);

                if (isRightAtApex || isInside)
                {
                    vRight = vPortalRight;
                    iRightIndex = i;
                }
                else
                {
                    // Left를 코너로 확정
                    PushPoint(vLeft);
                    vApex = vLeft;
                    iApexIndex = iLeftIndex;

                    vLeft = vRight = vApex;
                    iLeftIndex = iRightIndex = iApexIndex;

                    i = iApexIndex;
                    continue;
                }
            }
        }

        ++i;
    }

    PushPoint(ProjectXZ(vTargetPos)); // 중복 방지는 PushPoint가 처리

    return (_OutWaypoints.size() >= 2);
}

void CNavigation::Nudge_Waypoints_InsideCells(vector<_float3>& _OutWaypoints)
{
    // Find_Cell() 결과 불규칙 문제로 실제 사용하지 않음
    // → Nudge_Waypoints_AlongPath()로 대체
    float kNudgeDist = 0.5f;

    for (size_t i = 1; i + 1 < _OutWaypoints.size(); ++i)
    {
        _vector vWP = XMVectorSetW(XMLoadFloat3(&_OutWaypoints[i]), 1.0f);

        int cellIndex = Find_CellIndex(vWP);
        if (cellIndex == -1) continue;

        _vector vCenter = m_Cells[cellIndex]->Get_Center_Simd();
        vCenter = XMVectorSetY(vCenter, XMVectorGetY(vWP));

        _vector dir = XMVectorSetY(XMVectorSubtract(vCenter, vWP), 0.f);
        const float lenSq = XMVectorGetX(XMVector3LengthSq(dir));
        if (lenSq < 1e-8f) continue;

        dir = XMVector3Normalize(dir);
        vWP = XMVectorAdd(vWP, XMVectorScale(dir, kNudgeDist));
        XMStoreFloat3(&_OutWaypoints[i], vWP);
    }
}

// ------------------------------------------------------------
// Nudge_Waypoints_AlongPath
// Funnel 결과 Waypoint가 NavMesh 경계와 정확히 겹칠 경우
// 셀 판정에 의존하지 않고, 경로의 기하적 진행 방향(Prev-Cur-Next)을
// 기준으로 Waypoint를 미세 보정해 안정성을 확보한다
// ------------------------------------------------------------
void CNavigation::Nudge_Waypoints_AlongPath(vector<_float3>& _OutWaypoints)
{
    constexpr float kDirEpsSq = 1e-8f;
    constexpr float kStraightEps = 1e-4f; // 거의 직선 판정 임계값

    if (_OutWaypoints.size() < 3) return;

    for (size_t i = 1; i + 1 < _OutWaypoints.size(); ++i)
    {
        const _vector vPrevPos = XMLoadFloat3(&_OutWaypoints[i - 1]);
        _vector       vCurPos = XMLoadFloat3(&_OutWaypoints[i]);
        const _vector vNextPos = XMLoadFloat3(&_OutWaypoints[i + 1]);

        // prev → next 방향 (경로 전체 방향)
        _vector vPathDir = XMVectorSetY(XMVectorSubtract(vNextPos, vPrevPos), 0.f);
        if (XMVectorGetX(XMVector3LengthSq(vPathDir)) < kDirEpsSq) continue;
        vPathDir = XMVector3Normalize(vPathDir);

        // prev → cur 방향 (현재 구간 방향)
        _vector vSegDir = XMVectorSetY(XMVectorSubtract(vCurPos, vPrevPos), 0.f);
        if (XMVectorGetX(XMVector3LengthSq(vSegDir)) < kDirEpsSq) continue;
        vSegDir = XMVector3Normalize(vSegDir);

        // 회전 방향 부호 (LH, Y-up)
        const float fTurnSignY = XMVectorGetY(XMVector3Cross(vPathDir, vSegDir));

        if (fabsf(fTurnSignY) < kStraightEps) continue; // 직선 구간은 보정 생략

        // XZ 평면 좌/우 법선
        const _vector vLeftNormalXZ = XMVectorSet(-XMVectorGetZ(vPathDir), 0.f, XMVectorGetX(vPathDir), 0.f);
        const _vector vRightNormalXZ = XMVectorSet(XMVectorGetZ(vPathDir), 0.f, -XMVectorGetX(vPathDir), 0.f);

        // 회전 방향 쪽으로 Nudge
        const _vector vNudgeDir = (fTurnSignY < 0.f) ? vLeftNormalXZ : vRightNormalXZ;
        vCurPos = XMVectorAdd(vCurPos, XMVectorScale(vNudgeDir, m_fNudgeDist));
        XMStoreFloat3(&_OutWaypoints[i], vCurPos);
    }
}

const _float CNavigation::Compute_Heuristic(_int _iStartIndex, _int _iTargetIndex)
{
    return Dist3(m_Cells[_iStartIndex]->Get_Center_Simd(), m_Cells[_iTargetIndex]->Get_Center_Simd());
}

CNavigation* CNavigation::Create(ID3D11Device* _pDevice, ID3D11DeviceContext* _pContext, const _tchar* _pNavigationDataFile)
{
    CNavigation* pInstance = new CNavigation(_pDevice, _pContext);

    if (FAILED(pInstance->Initialize_Prototype(_pNavigationDataFile)))
    {
        MSG_BOX("Failed to Created : CNavigation");
        Safe_Release(pInstance);
    }
    return pInstance;
}

#ifdef _DEBUG
HRESULT CNavigation::Render()
{
    if (FAILED(m_pShader->Bind_Matrix("g_WorldMatrix", &m_WorldMatrix)))                                                           return E_FAIL;
    if (FAILED(m_pShader->Bind_Matrix("g_ViewMatrix", &m_pGameInstance->Get_TransformFloat4x4(CPipeLine::D3DTS_VIEW))))          return E_FAIL;
    if (FAILED(m_pShader->Bind_Matrix("g_ProjMatrix", &m_pGameInstance->Get_TransformFloat4x4(CPipeLine::D3DTS_PROJ))))          return E_FAIL;

    for (auto& pCell : m_Cells)
        if (nullptr != pCell) pCell->Render(m_pShader);

    for (int& idx : m_CellPath)
        if (m_Cells[idx]) m_Cells[idx]->Render(m_pShader);

    // Waypoint 디버그 렌더
    m_pEffect->SetWorld(XMMatrixIdentity());
    m_pEffect->SetView(m_pGameInstance->Get_TransformMatrix(CPipeLine::D3DTS_VIEW));
    m_pEffect->SetProjection(m_pGameInstance->Get_TransformMatrix(CPipeLine::D3DTS_PROJ));
    m_pEffect->Apply(m_pContext);
    m_pContext->IASetInputLayout(m_pInputLayout);

    m_pBatch->Begin();
    BoundingSphere sphere{};
    sphere.Radius = 0.5f;
    for (auto& Waypoint : m_Waypoints)
    {
        sphere.Center = Waypoint;
        DX::Draw(m_pBatch, sphere, XMVectorSet(1.0f, 1.0f, 0.0f, 1.0f));
    }
    m_pBatch->End();

    return S_OK;
}
#endif

CComponent* CNavigation::Clone(void* _pArg)
{
    CNavigation* pInstance = new CNavigation(*this);

    if (FAILED(pInstance->Initialize(_pArg)))
    {
        MSG_BOX("Failed to Cloned : CNavigation");
        Safe_Release(pInstance);
    }
    return pInstance;
}

void CNavigation::Free()
{
#ifdef _DEBUG
    Safe_Release(m_pShader);
    Safe_Release(m_pInputLayout);
    if (false == m_isCloned)
    {
        Safe_Delete(m_pBatch);
        Safe_Delete(m_pEffect);
    }
#endif

    for (auto& pCell : m_Cells)
        Safe_Release(pCell);
    m_Cells.clear();

    __super::Free();
}