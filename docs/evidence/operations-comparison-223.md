# AMR 운영 비교 보고서

- 원본 SHA-256: `25727b03b5e73c5107fc8926a87d7889904ae49851ae2561b81b6d1de31018ff`
- 비교 조건 SHA-256: `bae34ab1381aacf49f92e97e6346c36cee6e3a5c28e4d2511cb3380a882c8977`
- 작성 가정: 지평 5400.0초, lot 12건, 30.0초 간격 투입, 납기 여유 240.0초입니다.
- 평균 공정 WIP는 설비 입력·처리·출구 버퍼의 시간 평균입니다. 미투입 대기 주문 전체와 출하 백로그는 포함하지 않습니다.
- 교통 대기 합계는 여러 차량의 대기 시간을 더한 값이므로 시뮬레이션 지평보다 클 수 있습니다.
- 미완료는 작성된 지평 종료 시점에 작업이 남았다는 뜻이며, 이 보고서는 이를 교착으로 판정하지 않습니다.
- 벽시계 실행 시간은 외부 부하의 영향을 받으므로 정책 성능 개선율 근거로 사용하지 않습니다.
- `incomplete`, `failed`, `invalid` 상태와 오류는 아래 표에 그대로 표시합니다.

| 시나리오 | 정책 | 상태 | 소스 안정 | 처리 완료 | 빈 용기 반납 | 평균 공정 WIP | 교통 대기 합계(s) | 출하 지연 하한(s) | 최소 여유(m) | 최대 동시 이동 | 실행 ID |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---|
| normal | fifo | complete | 예 | 12 | 12 | 0.26 | 8256.30 | 17420.90 | 1.73 | 4 | `operations-normal-fifo-a418a2d74edbcbb1-30d5129ac8be` |
| normal | eta | complete | 예 | 12 | 12 | 0.30 | 4780.10 | 16317.90 | 1.73 | 4 | `operations-normal-eta-13b5cf02963534bc-3995585611fc` |
| tester_down | fifo | complete | 예 | 12 | 12 | 0.28 | 8121.10 | 18079.20 | 1.73 | 4 | `operations-tester_down-fifo-a17dcb2d042c7443-9ff6159288c7` |
| tester_down | eta | complete | 예 | 12 | 12 | 0.31 | 4290.30 | 17635.80 | 1.73 | 4 | `operations-tester_down-eta-400e7211865a3c5f-bb8cb10fb6d9` |
| aisle_closed | fifo | complete | 예 | 12 | 12 | 0.27 | 10137.10 | 19069.90 | 1.73 | 4 | `operations-aisle_closed-fifo-16d8a2f2b4ab2a5c-d07c7f07910c` |
| aisle_closed | eta | complete | 예 | 12 | 12 | 0.39 | 6655.50 | 21097.50 | 1.73 | 4 | `operations-aisle_closed-eta-68fe7a60caa18645-1b8101bb477e` |

## 실행 범위와 오류

- `normal/fifo`: scope=`authored deterministic CPU operations comparison; no physics or calibrated production claim`; business_finished=true; fleet_quiescent=true; error=`없음`
- `normal/eta`: scope=`authored deterministic CPU operations comparison; no physics or calibrated production claim`; business_finished=true; fleet_quiescent=true; error=`없음`
- `tester_down/fifo`: scope=`authored deterministic CPU operations comparison; no physics or calibrated production claim`; business_finished=true; fleet_quiescent=true; error=`없음`
- `tester_down/eta`: scope=`authored deterministic CPU operations comparison; no physics or calibrated production claim`; business_finished=true; fleet_quiescent=true; error=`없음`
- `aisle_closed/fifo`: scope=`authored deterministic CPU operations comparison; no physics or calibrated production claim`; business_finished=true; fleet_quiescent=true; error=`없음`
- `aisle_closed/eta`: scope=`authored deterministic CPU operations comparison; no physics or calibrated production claim`; business_finished=true; fleet_quiescent=true; error=`없음`
