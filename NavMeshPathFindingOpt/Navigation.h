#pragma once
#include "Component.h"

// Navigation Component
// 1. Navigation Cell 정보를 참조해 오브젝트의 이동 가능 여부를 제어
// 2. PathFinding 후 경로 정보를 반환

BEGIN(Engine)
class CCell;
class CShader;
typedef struct tagAStarNode
{
	_int index;
	_float F;
	_float G;
}AStarNode;

typedef struct tagNavigationPortal
{
	_float3 left;
	_float3 right;
}NavPortal;


typedef struct tagMinF
{
	bool operator()(const AStarNode& a, const AStarNode& b) const
	{
		// priority_queue는 "우선순위가 높은 것"이 top.
		// Min-heap처럼 쓰려면 a가 b보다 "뒤로 가야 할 때" true가 되게(= a.F > b.F)
		return a.F > b.F;
	}
}MinF;


class ENGINE_DLL CNavigation final :  public CComponent
{
public:
	typedef struct
	{
		_int	iStartIndex = { -1 };
		_float fNudgeDist = 1.0f;
	}NAVIGATION_DESC;

private:
	CNavigation(ID3D11Device* _pDevice, ID3D11DeviceContext* _pContext);
	CNavigation(const CNavigation& _Prototype);
	virtual ~CNavigation() = default;

public:
	virtual HRESULT			Initialize_Prototype(const _tchar* _pNavigationDataFile);
	virtual HRESULT			Initialize(void* _pArg);
	virtual void			Update(_fmatrix _vMapWorldmatrix);

public:
	void					Setting_CurCellIndex(_fvector _vOwnerWorldPos); // 삭제
	int						Find_CellIndex(_fvector _vOwnerWorldPos);

public: // 이동 가능 여부 
	_bool					Is_Move(_fvector _vOwnerWorldPos, _float3* _pOutLineNormal = nullptr);
	_vector					Adjust_CellHeight(_fvector _vPos);

#ifdef _DEBUG
public:
	virtual					HRESULT Render();
private:
	CShader*				m_pShader = nullptr;
	PrimitiveBatch<VertexPositionColor>* m_pBatch = nullptr;
	BasicEffect* m_pEffect = nullptr;
	ID3D11InputLayout* m_pInputLayout = nullptr;
#endif // Debug Render

public:
	const vector<CCell*>&	Get_Cells() const { return m_Cells; }
	_int					Get_CurCellIndex() const { return m_iCurCellIndex; }
	void					Set_CurCellIndex(_int _iCurCellIndex) { m_iCurCellIndex = _iCurCellIndex; }

private:
	vector<CCell*>			m_Cells;
	_int					m_iCellCount = 0;
	static _float4x4		m_WorldMatrix;
	_int					m_iCurCellIndex = -1;

private:
	HRESULT					SetUp_Neighbor();
	_float					Compute_Height(_int _iCurCellIndex, _fvector _vPos);


public: // PathFinding
	_bool					Build_PathTo(const _fvector _vStartPos, const _fvector _vTargetPos, vector<_float3>& _OutWaypoints);

private:
	_bool					Build_CellPath(const _fvector _vStartPos, const _fvector _vTargetPos);
	const _float			Compute_Heuristic(_int _iStartIndex, _int _iTargetIndex);

	_bool					Build_Waypoints_CellCentroid(const _fvector _vStartPos, const _fvector _vTargetPos, vector<_float3>& _OutWaypoints);
	_bool					Build_Waypoints_SimpleStupidFunnel(const _fvector _vStartPos, const _fvector _vTargetPos, vector<_float3>& _OutWaypoints);
	_bool					Build_PortalList();
	_bool					Run_Funnel(const _fvector _vStartPos, const _fvector _vTargetPos, vector<_float3>& _OutWaypoints);

	void					Nudge_Waypoints_InsideCells(vector<_float3>& _OutWaypoints);
	void					Nudge_Waypoints_AlongPath(vector<_float3>& _OutWaypoints);
	
	// A*
	vector<_float>			m_gCost;
	vector<_int>			m_Parents;
	vector<_int>			m_CellPath;
	vector<_uint>			m_Stamps;			// A* 초기화 비용 생략 + ClosedList 역할					
	_uint					m_iCurStamp = 0;

	// Simple Stupid Funnel
	vector<_int>			m_ParentExitLines;  // parent(cur)에서 child(next)로 나가는 라인 캐싱 -> 퍼넬 알고리즘에서 사용
	vector<_int>			m_PortalLines;
	vector<NavPortal>		m_Portals;

	// Nudge
	_float					m_fNudgeDist = 1.0f;

	// Result
	vector<_float3>			m_Waypoints;

	inline _float			Dist3(const _fvector a, const _fvector b)
	{
		return XMVectorGetX(XMVector3Length(XMVectorSubtract(a, b)));
	}

	inline _float			TriArea2(const _fvector a, const _fvector b, const _fvector c)
	{
		const _vector ab = XMVectorSubtract(b, a);
		const _vector ac = XMVectorSubtract(c, a);
		return XMVectorGetY(XMVector3Cross(ab, ac));
	}

public:
	static CNavigation* Create(ID3D11Device* _pDevice, ID3D11DeviceContext* _pContext, const _tchar* _pNavigationDataFile);
	virtual CComponent* Clone(void* _pArg) override;
	virtual void Free() override;
};

END
