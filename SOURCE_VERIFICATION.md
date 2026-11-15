# 공개본 변경 및 검증 내역

종목1·종목2는 공개용 RFID 처리를 수정했으므로 원본과 바이트가 다릅니다. Calibration은 원본과 SHA-256이 일치합니다.

## 변경 범위

- 실제 UID 배열 → 0으로 채운 자리표시자 및 기본 false인 설정 플래그.
- 종목1·종목2 모두 설정 플래그가 true이고 카드 UID가 일치할 때만 출발 허용.
- 실제 UID가 적힌 주석 정리 및 종목1 카드 UID 시리얼 출력 제거.
- RFID 함수와 관련 주석을 제외한 코드는 원본과 일치함을 비교 확인.
- 스케치 폴더명과 .ino 파일명 일치 확인.
- 컴파일 및 실기 주행은 이번 작업에서 수행하지 않음.

## 공개본 SHA-256

| 파일 | SHA-256 |
| --- | --- |
| Mission1/Mission1.ino | `C762AF4D0F433DD324CB01E28F3C852BF2C8AEFC59FD6A85DEAF8353590C636B` |
| Mission2/Mission2.ino | `6E75653A36E7CF79E6658FE5C8487F6106ACA41410134F63D7776F0709598F7A` |
| Mission2Calibration/Mission2Calibration.ino | `59CFA853F9F5C636132CBA28A63AEAF0E40B93B88F79887CEB0096EF938CCF92` |
