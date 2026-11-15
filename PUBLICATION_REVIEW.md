# 공개용 정리 결과

- 종목1·종목2의 실제 RFID 카드 UID를 배열과 모든 관련 주석에서 제거했습니다.
- 공개본은 0으로 채운 자리표시자와 `RFID_UID_CONFIGURED = false`를 사용합니다. 0으로 된 카드도 허용하지 않습니다.
- 종목1의 인식 카드 UID 시리얼 출력을 제거했습니다.
- 카드 출발 기능은 유지됩니다. 본인의 4바이트 UID를 로컬에서 입력하고 설정 플래그를 true로 바꾼 후 업로드하세요.
- 주행 경로, 모터·센서·리프트 보정값은 변경하지 않았습니다. Calibration은 원본 그대로입니다.
- 소스에서 비밀번호, API 토큰, 개인 이메일·전화번호·로컬 경로는 발견하지 못했습니다.

실제 UID를 입력한 파일은 다시 공개하지 마세요. `.gitignore`는 `.ino` 안에 적은 값을 숨겨 주지 않습니다.

이번 파일 교체는 GitHub의 과거 커밋을 지우지 않습니다. 이전 UID가 이미 올라갔다면 저장소를 Private으로 전환하고, 이전 기록을 별도로 정리하거나 새 공개 저장소에 이 파일들만 업로드하세요. 다른 사람이 이미 만든 복사본까지 회수할 수는 없습니다.

[GitHub 공식 기록 정리 안내](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/removing-sensitive-data-from-a-repository)