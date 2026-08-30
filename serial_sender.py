from dataclasses import dataclass
import time

try: 
    import serial
except ImportError:
    serial=None

VALID_STATES={"IDLE", "LOW", "MID", "HIGH", "ESCAPE"}

@dataclass(frozen=True)
class SerialConfig:
    port: str="/dev/ttyUSB0" #라즈베리 파이 포트
    baudrate: int=9600 #통신 속도
    timeout_s: float=1.0 #시리얼 통신 대기 시간
    reset_wait_s: float=2.0 #아두이노 연결 후 기다리는 시간
    dry_run: bool=True #테스트 모드
   
#파이에서 아두이노로 상태 메시지 보내는 클래스 
class SerialStateSender:
    #config 들어오면 그 설정 사용, 없으면 기본값
    def __init__(self, config: SerialConfig | None=None):
        self.config=config or SerialConfig()
        #마지막에 보낸 상태 저장
        self.last_sent_state: str | None=None
        self.ser=None
        
    def connect(self)->None:
        #dry run이면 연결x
        if self.config.dry_run:
            print("[DRY RUN] Serial connection skipped")
            return
        if serial is None:
            raise RuntimeError(
                "install pyserial"
            )
        #연결합시다
        self.ser=serial.Serial(
            port=self.config.port,
            baudrate=self.config.baudrate,
            timeout=self.config.timeout_s,
        )
        time.sleep(self.config.reset_wait_s)
        
    def send_state(self, state: str)->bool:
        state=state.upper()
        
        if state not in VALID_STATES:
            raise ValueError(f"Invalid state: {state}")
        
        #이전과 같은 상태면 다시 보내지 않음
        if state == self.last_sent_state:
            return False
            
        message=f"{state}\n"
            
        if self.config.dry_run:
            print(f"[DRY RUN] send: {message.strip()}")
                
        else:
            if self.ser is None:
                raise RuntimeError("Serial is not connected. Call connect() first.")
            self.ser.write(message.encode("ascii")) #메세지를 아스키 바이트로 전환해서 보냄
            self.ser.flush() #버퍼에 남아 있는 데이터 바로 내보냄
                
        self.last_sent_state=state 
        return True #true 반환: 실제로 새 상태 전송했다는 뜻
            
    #시리얼 연결 닫음
    def close(self) -> None:
        if self.ser is not None:
            self.ser.close()
            self.ser=None
                    
    def __enter__(self):
        self.connect()
        return self
        
    def __exit__(self, exc_type, exc, traceback):
        self.close()
        
#테스트    
def run_demo()->None:
    sender=SerialStateSender(
        SerialConfig(
            dry_run=True
        )
    )
            
    sender.connect()
            
    states=["IDLE", "LOW", "LOW", "MID", "HIGH", "ESCAPE", "ESCAPE"]
            
    for state in states:
        sent=sender.send_state(state)
        print(f"state={state:<4} sent={sent}")
                
    sender.close()
                
if __name__=="__main__":
    run_demo()
                        
            
