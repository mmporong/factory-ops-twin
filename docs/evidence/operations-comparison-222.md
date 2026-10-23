# AMR 운영 비교 보고서

- 원본 SHA-256: `725c9fa10dbcd8a379492471131e2f45532a48ad01537c9fe66d19980da32367`
- 비교 조건 SHA-256: `2185b3e53f373035796ac99c1df88f4dfdbbba840173cb9ac0640494852f9b47`
- 작성 가정: 지평 3600.0초, lot 12건, 30.0초 간격 투입, 납기 여유 240.0초입니다.
- 평균 공정 WIP는 설비 입력·처리·출구 버퍼의 시간 평균입니다. 미투입 대기 주문 전체와 출하 백로그는 포함하지 않습니다.
- 교통 대기 합계는 여러 차량의 대기 시간을 더한 값이므로 시뮬레이션 지평보다 클 수 있습니다.
- 미완료는 작성된 지평 종료 시점에 작업이 남았다는 뜻이며, 이 보고서는 이를 교착으로 판정하지 않습니다.
- 벽시계 실행 시간은 외부 부하의 영향을 받으므로 정책 성능 개선율 근거로 사용하지 않습니다.
- `incomplete`, `failed`, `invalid` 상태와 오류는 아래 표에 그대로 표시합니다.

| 시나리오 | 정책 | 상태 | 소스 안정 | 처리 완료 | 빈 용기 반납 | 평균 공정 WIP | 교통 대기 합계(s) | 출하 지연 하한(s) | 최소 여유(m) | 최대 동시 이동 | 실행 ID |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---|
| normal | fifo | incomplete | 예 | 12 | 4 | 0.38 | 6949.70 | 17420.90 | 1.73 | 4 | `operations-normal-fifo-14b828f566cd38be-088a702683ff` |
| normal | eta | complete | 예 | 12 | 12 | 0.44 | 4780.10 | 16317.90 | 1.73 | 4 | `operations-normal-eta-683d6873991dfeac-e3cebf020000` |
| tester_down | fifo | incomplete | 예 | 12 | 4 | 0.42 | 6604.60 | 18079.20 | 1.73 | 4 | `operations-tester_down-fifo-daeb88f7097f2c50-dc3ff2150643` |
| tester_down | eta | incomplete | 예 | 10 | 9 | 0.48 | 7372.10 | 18798.40 | 1.73 | 4 | `operations-tester_down-eta-8a3cbbbcb57474a8-38af20402dc2` |
| aisle_closed | fifo | incomplete | 예 | 11 | 2 | 0.39 | 7503.00 | 18996.10 | 1.73 | 4 | `operations-aisle_closed-fifo-040d2bf5fc0fedc6-0648ca12251c` |
| aisle_closed | eta | incomplete | 예 | 10 | 9 | 0.57 | 6328.00 | 20921.60 | 1.73 | 4 | `operations-aisle_closed-eta-3fb55943b80a8616-a3c7d6507a4c` |

## 실행 범위와 오류

- `normal/fifo`: scope=`authored deterministic CPU operations comparison; no physics or calibrated production claim`; business_finished=false; fleet_quiescent=false; error=`없음`
- `normal/eta`: scope=`authored deterministic CPU operations comparison; no physics or calibrated production claim`; business_finished=true; fleet_quiescent=true; error=`없음`
- `tester_down/fifo`: scope=`authored deterministic CPU operations comparison; no physics or calibrated production claim`; business_finished=false; fleet_quiescent=false; error=`없음`
- `tester_down/eta`: scope=`authored deterministic CPU operations comparison; no physics or calibrated production claim`; business_finished=false; fleet_quiescent=false; error=`없음`
- `aisle_closed/fifo`: scope=`authored deterministic CPU operations comparison; no physics or calibrated production claim`; business_finished=false; fleet_quiescent=false; error=`없음`
- `aisle_closed/eta`: scope=`authored deterministic CPU operations comparison; no physics or calibrated production claim`; business_finished=false; fleet_quiescent=false; error=`없음`
