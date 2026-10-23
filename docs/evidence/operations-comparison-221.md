# AMR 운영 비교 보고서

- 원본 SHA-256: `7f7e43ba3233c9b6a8bf8fb4ce5bdacb220763c67b8b777b43a9c091692ec4e3`
- 비교 조건 SHA-256: `a6fb01cfa6e8d8bbe69382a6cf3408fb50cf6fb4c2a5ed10ede45cad30da2d1f`
- 작성 가정: 지평 3600.0초, lot 12건, 30.0초 간격 투입, 납기 여유 240.0초입니다.
- 평균 공정 WIP는 설비 입력·처리·출구 버퍼의 시간 평균입니다. 미투입 대기 주문 전체와 출하 백로그는 포함하지 않습니다.
- 교통 대기 합계는 여러 차량의 대기 시간을 더한 값이므로 시뮬레이션 지평보다 클 수 있습니다.
- 미완료는 작성된 지평 종료 시점에 작업이 남았다는 뜻이며, 이 보고서는 이를 교착으로 판정하지 않습니다.
- 벽시계 실행 시간은 외부 부하의 영향을 받으므로 정책 성능 개선율 근거로 사용하지 않습니다.
- `incomplete`, `failed`, `invalid` 상태와 오류는 아래 표에 그대로 표시합니다.

| 시나리오 | 정책 | 상태 | 소스 안정 | 처리 완료 | 빈 용기 반납 | 평균 공정 WIP | 교통 대기 합계(s) | 출하 지연 하한(s) | 최소 여유(m) | 최대 동시 이동 | 실행 ID |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---|
| normal | fifo | incomplete | 예 | 12 | 4 | 0.38 | 6939.70 | 17355.70 | 1.73 | 4 | `operations-normal-fifo-c9760353ca0ab704-723b492b5fa5` |
| normal | eta | incomplete | 예 | 12 | 12 | 0.43 | 4769.40 | 17429.40 | 1.73 | 4 | `operations-normal-eta-ca529073a0d8e850-79ed3ea87c4f` |
| tester_down | fifo | incomplete | 예 | 12 | 4 | 0.42 | 6603.00 | 18106.80 | 1.73 | 4 | `operations-tester_down-fifo-a5443fd2166f17f7-3474c64f610c` |
| tester_down | eta | complete | 예 | 12 | 12 | 0.44 | 3622.80 | 14992.40 | 1.73 | 4 | `operations-tester_down-eta-5250a101d44fe3a5-c7193d0cc3de` |
| aisle_closed | fifo | incomplete | 예 | 11 | 2 | 0.39 | 7351.20 | 18897.20 | 1.73 | 4 | `operations-aisle_closed-fifo-c4d9fd2a968604cd-44d01bd80e02` |
| aisle_closed | eta | incomplete | 예 | 4 | 3 | 0.15 | 11478.20 | 27242.60 | 1.73 | 4 | `operations-aisle_closed-eta-08af6a76f08099cf-a5c9aaa15abe` |

## 실행 범위와 오류

- `normal/fifo`: scope=`authored deterministic CPU operations comparison; no physics or calibrated production claim`; business_finished=false; fleet_quiescent=false; error=`없음`
- `normal/eta`: scope=`authored deterministic CPU operations comparison; no physics or calibrated production claim`; business_finished=true; fleet_quiescent=false; error=`없음`
- `tester_down/fifo`: scope=`authored deterministic CPU operations comparison; no physics or calibrated production claim`; business_finished=false; fleet_quiescent=false; error=`없음`
- `tester_down/eta`: scope=`authored deterministic CPU operations comparison; no physics or calibrated production claim`; business_finished=true; fleet_quiescent=true; error=`없음`
- `aisle_closed/fifo`: scope=`authored deterministic CPU operations comparison; no physics or calibrated production claim`; business_finished=false; fleet_quiescent=false; error=`없음`
- `aisle_closed/eta`: scope=`authored deterministic CPU operations comparison; no physics or calibrated production claim`; business_finished=false; fleet_quiescent=false; error=`없음`
