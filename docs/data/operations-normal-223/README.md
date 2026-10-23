# 223 정상 운영 기록 재생 패키지

이 폴더에는 `operations-normal-eta-13b5cf02963534bc-ea817f1064d0` 실행에서 고른 관제 표본 120개가 들어 있다. 각 표본은 원본 JSON을 보간하거나 다시 계산하지 않고 그대로 담았다. 웹에서는 읽기 전용 운영 기록으로 재생되며, 물리 시뮬레이션을 다시 실행하는 자료는 아니다.

원본 실행 전체는 출하 12건과 빈 토트 반납 12건을 마친 상태다. 이 패키지가 담은 1980.1초부터 2218.1초까지의 구간은 화면 재생용 발췌본이므로, 발췌 구간만으로 전체 실행의 완료를 입증하지 않는다. 파일 해시와 선택 범위는 [provenance.json](provenance.json)에 기록했다.

## 파일 복원

PowerShell에서 저장소 루트로 이동한 뒤 압축 파일을 관제 API의 시나리오 폴더에 복원한다. 복원되는 JSON 파일명은 가져오기 단계에서 만든 이름과 같다.

```powershell
cd "$HOME/factory-ops-twin"
New-Item -ItemType Directory -Force "runs/operator-scenarios" | Out-Null
@'
import gzip
from pathlib import Path

source = Path("docs/data/operations-normal-223/223-normal-operations-1980-2220-9802772714a0.json.gz")
target = Path("runs/operator-scenarios/223-normal-operations-1980-2220-9802772714a0.json")
with gzip.open(source, "rb") as stream:
    target.write_bytes(stream.read())
print(target)
'@ | python -
```

복원본의 SHA-256은 `1bc685d5a583b68bb76df589c2b2fe8186f526413fcd63930ebf4a7ecaf1e140`이다. 아래 명령으로 확인할 수 있다.

```powershell
Get-FileHash -Algorithm SHA256 "runs/operator-scenarios/223-normal-operations-1980-2220-9802772714a0.json"
```

## 웹에서 재생

API를 반도체 후공정 배치와 복원한 시나리오 폴더로 시작한다.

```powershell
cd "$HOME/factory-ops-twin"
uv run python -m factory_ops.dashboard --config configs/factory-semiconductor-dual-lane --scenario-dir runs/operator-scenarios --port 8765
```

웹 서버가 실행 중이지 않을 때만 다른 PowerShell에서 시작한다.

```powershell
cd "$HOME/factory-ops-twin/dashboard"
npm run dev -- --port 3007
```

`http://127.0.0.1:3007`을 열고 `기록 재생` 패널의 `저장된 기록`에서 `223 normal operations 1980-2220`을 고른 뒤 `기록 불러오기`를 누른다. 이전·다음 버튼, 슬라이더, `표본 재생`으로 120개 관측을 확인할 수 있다. 재생 중 차량 조작은 비활성화된다.

8765 API나 3007 웹 서버가 이미 실행 중이면 위 명령으로 같은 포트에 프로세스를 더 띄우지 않는다. 기존 API가 이미 `runs/operator-scenarios`를 `--scenario-dir`로 사용한다면 웹에서 목록을 새로 확인하면 된다. 해당 옵션 없이 시작한 API라면 사용 중인 작업을 마친 뒤 그 API만 종료하고 위 명령으로 다시 시작한다. API 주소가 그대로라면 웹 서버는 다시 시작할 필요가 없다.

## 공개 범위

공개 묶음에는 이 120개 표본의 gzip 파일과 출처 기록만 포함한다. 원본 5,400개 표본, 별도 GPU 900프레임 자료, SDK 설치 파일, USD 에셋, 이미지와 영상은 포함하지 않는다. 복원한 JSON의 필드와 문자열을 검사했으며 개인 이름, 이메일, 전화번호, 로컬 절대경로, IP 주소, 자격증명은 발견되지 않았다. 로봇·작업·주문·토트·설비 식별자는 시뮬레이션 운영용 합성 값이다.
