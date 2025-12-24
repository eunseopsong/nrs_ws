### Gravity Compensation
- Python G_Comp_Matrix.py로 추정한 값을 보고
- TOOL_MASS 에 질량 [kg]
- TOOL_COG 에 FT frame 기준 CoM [m]를 채워주면 된다.
- 예시  
    static double TOOL_MASS = 1.234;  
    YMatrix TOOL_COG =  
    {  
        { 0.015 },  // r_x  
        { 0.002 },  // r_y  
        { 0.120 }   // r_z  
    };  

- GRAVITY_COMPENSATION_MODE 를 1로 두기  

## AFT200-D80-C (CAN 이용 센서 셋팅 방법)
### CAN ID 바꾸기
- MeChanJshim GIT에서 “NRS_FT_AQ” 다운
- 거기에서 “CAN_sender”를 사용할 것임
- 실행하면 “현재 CAN ID (DEC)”를 입력하고
- 첫번째 데이터에 “1(DEC)”를 입력
- 두번째 데이터에 “바꿀 ID (DEC)”를 입력하고
- 나머지 데이터에다가는 “0”을 입력하고 보냄

<img width="1536" height="504" alt="image" src="https://github.com/user-attachments/assets/2ec537a5-c1e4-4808-9465-024e709d4a2d" />

- 이후 바꾼 아이디 입력하고 첫번째는 “3”, 두번째는 “1”입력하면
- 데이터 나옴

### CAN Sampling frequency 바꾸기
<img width="1536" height="504" alt="image" src="https://github.com/user-attachments/assets/f5557e0e-717d-4fd9-b4fe-278c03903bbb" />

- 여기에서 Data field [1]에 있던 내용을 “0x05”로 쓰고  
- “Data field [2]: 0x03, Data field [3]: 0xE8” → Ts: 1 ms (1,000 Hz) (내가 만든 프로그램에선 DEC로 쓰기에 “03, 232” 로 작성)  
- “Data field [2]: 0x07, Data field [3]: 0xD0” → Ts: 2 ms (500 Hz) (내가 만든 프로그램에선 DEC로 쓰기에 “07, 208” 로 작성)  
* 여기서 설정하는 건 Sampling frequency가 아니라 Sampling time이고 단위는 nano-second  
* 그리고 주파수 새로 설정 시엔 센서 선 뺐다가 다시 끼우고 하는 게 좋음  

### 공장 초기화
<img width="1967" height="732" alt="image" src="https://github.com/user-attachments/assets/238b89b8-4b4e-4798-ba3b-d39c034c859e" />

- ID: FF, Data: FF,0,0,0,0,0,0  
- 이후에 아래 커맨드 날려야 값나옴  
- ID: 102(16진법), 258(10진법, 터미널에는 이걸로)  
  DATA: 3, 1, 0, 0, 0, 0, 0  
