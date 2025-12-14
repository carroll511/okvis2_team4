#!/bin/bash
# 스왑 공간을 32GB로 확대하는 스크립트
# 호스트 시스템에서 root 권한으로 실행

set -e

echo "현재 스왑 상태:"
swapon --show
free -h

echo ""
echo "기존 스왑 파일 비활성화 및 제거..."
sudo swapoff /swapfile 2>/dev/null || true
sudo rm -f /swapfile

echo "새로운 32GB 스왑 파일 생성 중 (시간이 걸릴 수 있습니다)..."
sudo fallocate -l 64G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

echo ""
echo "스왑 파일을 /etc/fstab에 추가..."
if ! grep -q "/swapfile" /etc/fstab; then
    echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
else
    echo "/swapfile이 이미 /etc/fstab에 있습니다."
    # 기존 라인 업데이트
    sudo sed -i 's|^/swapfile.*|/swapfile none swap sw 0 0|' /etc/fstab
fi

echo ""
echo "스왑 우선순위 조정 (메모리 우선 사용)..."
if ! grep -q "vm.swappiness" /etc/sysctl.conf; then
    echo "vm.swappiness=10" | sudo tee -a /etc/sysctl.conf
    sudo sysctl vm.swappiness=10
else
    echo "vm.swappiness가 이미 설정되어 있습니다."
    sudo sysctl vm.swappiness=10
fi

echo ""
echo "완료! 새로운 스왑 상태:"
swapon --show
free -h

echo ""
echo "총 사용 가능 메모리:"
TOTAL_MEM=$(free -g | awk '/^Mem:/ {print $2}')
TOTAL_SWAP=$(swapon --show=SIZE --noheadings | sed 's/G//' | awk '{sum+=$1} END {print sum}')
echo "물리 메모리: ${TOTAL_MEM}GB"
echo "스왑: ${TOTAL_SWAP}GB"
echo "총: $((TOTAL_MEM + TOTAL_SWAP))GB"
echo ""
echo "참고: 시스템을 재부팅해도 스왑이 자동으로 활성화됩니다."

