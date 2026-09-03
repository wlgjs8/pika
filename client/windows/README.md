# PIKA Windows client

이 폴더는 Windows 기본 OpenSSH와 브라우저만 사용해 원격 수집과 저지연 프리뷰를
한 번에 실행합니다. Python이나 X server는 필요하지 않습니다.

1. `setup_ssh_key.cmd`를 실행합니다.
   - 처음 실행하면 `.env`를 만들고 메모장을 엽니다.
   - `PIKA_HOST`, `PIKA_SSH_USER`, `PIKA_REMOTE_DIR`를 확인하고 저장합니다.
   - 스크립트가 자동 실행 전용 키를 별도로 만들며, 원격 계정 비밀번호는 공개키를
     등록할 때 한 번만 입력합니다.
2. 이후에는 `start_collect_preview.cmd`를 실행합니다.
   - 원격 수집이 시작되고 준비가 끝나면 브라우저가 자동으로 열립니다.
   - 콘솔에서 `b`는 녹화 시작/정지, `Ctrl+C`는 마지막 저장 후 종료입니다.

SSH 비밀번호를 `.env`에 넣지 마세요. 영상이 불안정하면 `.env`의
`PIKA_PREVIEW_FPS=10`을 `5`로 낮추면 됩니다.
