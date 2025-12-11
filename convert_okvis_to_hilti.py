import pandas as pd
import sys
import os


# ================= 설정 영역 =================
# OKVIS2 컬럼 -> 목표 형식 컬럼 매핑
# 입력 파일의 헤더를 보고 작성했습니다.
column_mapping = {
    "timestamp": "timestamp_s",
    "p_WS_W_x": "tx",
    "p_WS_W_y": "ty",
    "p_WS_W_z": "tz",
    "q_WS_x": "qx",
    "q_WS_y": "qy",
    "q_WS_z": "qz",
    "q_WS_w": "qw",
}
# ===========================================


def convert_csv_to_txt(input_csv, output_txt):
    try:
        # 1. CSV 파일 읽기
        df = pd.read_csv(input_csv, index_col=False, skipinitialspace=True)
        df.columns = df.columns.str.strip()

        print(f"CSV 파일을 불러왔습니다. 행 개수: {len(df)}")

        # 2. 컬럼 이름 변경 및 선택
        df = df.rename(columns=column_mapping)
        target_columns = ["timestamp_s", "tx", "ty", "tz", "qx", "qy", "qz", "qw"]

        # 컬럼 존재 확인
        if not all(col in df.columns for col in target_columns):
            print(f"오류: 필수 컬럼이 누락되었습니다.")
            return

        df = df[target_columns]

        # 3. 타임스탬프 단위 변환
        if df["timestamp_s"].iloc[0] > 1e18:
            print("타임스탬프 단위 변환 (ns -> s)")
            df["timestamp_s"] = df["timestamp_s"] / 1e9

        # ==========================================================
        # [수정된 부분] 헤더와 데이터를 분리해서 저장
        # ==========================================================

        # 4. 파일을 쓰기 모드('w')로 열어서 헤더를 직접 작성
        with open(output_txt, "w") as f:
            f.write("# timestamp_s tx ty tz qx qy qz qw\n")

        # 5. Pandas로 데이터만 이어붙이기 ('a' 모드 = append)
        # header=False: Pandas가 헤더를 쓰지 못하게 함 (따옴표 문제 해결)
        df.to_csv(
            output_txt,
            sep=" ",
            index=False,
            header=False,
            mode="a",
            float_format="%.9f",
        )

        print(f"\n변환 성공! '{output_txt}' 파일이 저장되었습니다.")

        # 결과 미리보기
        print("\n[저장된 파일 내용 미리보기]")
        with open(output_txt, "r") as f:
            for _ in range(3):
                print(f.readline().strip())

    except FileNotFoundError:
        print(f"오류: 파일을 찾을 수 없습니다.")
    except Exception as e:
        print(f"오류 발생: {e}")


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("사용법: python3 transform_csv_to_txt.py [input_csv] [output_txt]")
        sys.exit(1)

    convert_csv_to_txt(sys.argv[1], sys.argv[2])
