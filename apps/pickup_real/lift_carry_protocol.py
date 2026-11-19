"""Carry 전용 지게팔 응답 해석. 장치 접근 없음; 공유 LiftUno 구현은 수정하지 않는다."""
import re

PULSES,CHECK=12800,320


def suffix_line(line,expected):
    line=line.strip()
    return line==expected or any(line==expected[n:] for n in range(1,min(4,len(expected)-5)))


def is_reboot(line):
    low=line.lower()
    return "ready" in low or "redy" in low or "u: up (ultrasonic" in low


def banner_line(line,expected):
    return suffix_line(line,expected) or (expected=="Motor ready" and ("ready" in line.lower() or "redy" in line.lower()))


def parse_record(rec,stop_cm,nominal=None):
    lines=rec["lines"]
    start=next((t for t,l in lines if suffix_line(l,"Up start")),None)
    reboot=any(is_reboot(l) for t,l in lines if t>=rec.get("sent",0) and (start is None or t>start))
    serial_error=any("SERIAL_ERROR" in l for _,l in lines)
    echoes=[]
    for t,line in lines:
        match=re.search(r"Distance:\s*(-?\d+(?:\.\d+)?)\s*cm",line,re.I)
        if match and float(match.group(1))>0:
            echoes.append((t,float(match.group(1))))
    out=dict(result=rec.get("result"),error=rec.get("error"),reboot=reboot,serial_error=serial_error,
             echo_cm=[round(cm,2) for _,cm in echoes],t_start=start,end=None)
    if start is None or reboot or serial_error:
        return out
    end=None
    for t,line in lines:
        if t<start:
            continue
        if suffix_line(line,"Up complete"):
            end=(t,"complete","completion line")
            break
        if suffix_line(line,"Up stopped: distance below stop threshold"):
            end=(t,"height_limit","height-limit line")
            break
    now=rec.get("now",max([t for t,_ in lines]+[start]))
    pre=echoes[0] if echoes and echoes[0][0]-start<=.16 else None
    moving=[e for e in echoes if e is not pre and e[0]>=start]
    # 끝 줄이 없으면 시간만으로 성공 처리하지 않는다. 마지막 거리·완전한 확인 수·침묵을 함께 본다.
    if end is None and echoes and now-echoes[-1][0]>=.45:
        if echoes[-1][1]<stop_cm:
            end=(echoes[-1][0],"height_limit","height-limit echo + quiet")
        elif nominal and pre and len(moving)>=39 and now-start>=.10+PULSES*nominal+.3:
            end=(start+.10+PULSES*nominal,"complete","40 distance checks + elapsed + quiet")
    if end is None:
        return out
    t_end,kind,source=end
    out.update(end=kind,end_source=source,seconds=round(t_end-start,3),pre_cm=None if pre is None else round(pre[1],2))
    counts = [int(match.group(1)) for t, line in lines if t >= t_end
              for match in [re.fullmatch(r"Up pulses: (\d+)", line)] if match]
    if counts:
        pulses = counts[0]
        valid = (len(counts) == 1 and
                 (pulses == PULSES if kind == "complete"
                  else 0 <= pulses < PULSES and pulses % CHECK == 0))
        if valid:
            out.update(pulses=pulses, pulse_source="board count")
            if kind == "height_limit":
                out["stop_cm"] = round(echoes[-1][1], 2) if echoes else None
        return out
    if kind=="complete":
        out.update(pulses=PULSES,pulse_source="complete line" if source=="completion line" else source)
        return out
    out["stop_cm"]=round(echoes[-1][1],2) if echoes else None
    n=len([e for e in moving if e[0]<=t_end+.05])
    by_echo=n*CHECK
    by_time=None if not nominal else round(max(0.,t_end-start-.10)/nominal/CHECK)*CHECK
    out.update(pulses_by_echo=by_echo,pulses_by_time=by_time)
    if by_time is not None and abs(by_time-by_echo)>2*CHECK:
        pulses,source=by_time,"time (echo lines missing)"
    else:
        pulses,source=by_echo,"echo checks x 320"
    out.update(pulses=max(0,min(PULSES,int(pulses))),pulse_source=source)
    return out
