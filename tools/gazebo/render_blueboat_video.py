#!/usr/bin/env python3
"""Render a labelled MP4 reconstruction from independently recorded Gazebo poses.

This is NOT a screen capture, a real vessel camera, or an ArduPilot EKF trace.
The video can only be built from a previously verified BlueBoat trial.
"""
import argparse
import bisect
import json
import math
import subprocess
import sys
import tempfile
from pathlib import Path

from verify_blueboat_trial import load_truth


WIDTH, HEIGHT = 1080, 640
PLOT = (55, 112, 790, 565)


def font(size):
    from PIL import ImageFont

    for name in ("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
                 "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf"):
        if Path(name).is_file():
            return ImageFont.truetype(name, size)
    return ImageFont.load_default()


def render(truth_path, mission_path, verification_path, output_path,
           frames=240, fps=12):
    from PIL import Image, ImageDraw

    if not 24 <= frames <= 720 or not 5 <= fps <= 30:
        raise ValueError("frames must be 24..720 and fps must be 5..30")
    truth = load_truth(truth_path)
    mission = json.loads(Path(mission_path).read_text(encoding="utf-8"))
    verified = json.loads(Path(verification_path).read_text(encoding="utf-8"))
    if (not mission.get("pass") or not mission.get("armed_heartbeat_observed") or
        not mission.get("auto_heartbeat_observed") or
        verified.get("truth_source") != "Gazebo BlueBoat actual model odometry" or
        verified.get("not_rover_simstate_only") is not True):
        raise ValueError("cannot render unverified Gazebo BlueBoat trial")
    mode = mission.get("bridge_mode")
    if mode not in ("sensor-eskf", "sensor-blackout",
                    "onboard-route", "onboard-blackout"):
        raise ValueError("unexpected BlueBoat experiment mode")
    is_hold = mode.endswith("blackout")
    if is_hold != bool(verified.get("hold_confirmed")):
        raise ValueError("inconsistent verified HOLD trial")
    if not is_hold and verified.get("route_confirmed") is not True:
        raise ValueError("route completion was not verified")

    t = [p["monotonic_s"] for p in truth]
    if len(truth) < 30 or t[-1]-t[0] < 1.0:
        raise ValueError("insufficient Gazebo truth samples")
    hold_at = mission.get("hold_confirmed_monotonic_s") if is_hold else None
    if is_hold and (not isinstance(hold_at, (int, float)) or
                    not math.isfinite(hold_at) or not t[0] <= hold_at <= t[-1]):
        raise ValueError("HOLD timestamp outside independent Gazebo truth")

    xs = [p["x"] for p in truth]
    ys = [p["y"] for p in truth]
    extent = max(max(xs)-min(xs), max(ys)-min(ys), 4.0)
    # Use the same metres-per-pixel in X and Y: a circular motion stays round.
    scale = min((PLOT[2]-PLOT[0]-80)/extent,
                (PLOT[3]-PLOT[1]-80)/extent)
    center_x = (max(xs)+min(xs))*0.5
    center_y = (max(ys)+min(ys))*0.5

    def pos(point):
        return (round((PLOT[0]+PLOT[2])*0.5+(point["x"]-center_x)*scale),
                round((PLOT[1]+PLOT[3])*0.5-(point["y"]-center_y)*scale))

    coords = [pos(p) for p in truth]
    hold_index = bisect.bisect_left(t, hold_at) if is_hold else None
    title_font, body_font, small_font = font(24), font(16), font(13)
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)

    with tempfile.TemporaryDirectory(prefix="boat-video-frames-") as directory:
        root = Path(directory)
        for i in range(frames):
            current = t[0]+(t[-1]-t[0])*i/(frames-1)
            end = max(1, min(len(t), bisect.bisect_right(t, current)))
            image = Image.new("RGB", (WIDTH, HEIGHT), "#f6f9fc")
            draw = ImageDraw.Draw(image)
            draw.rectangle((0, 0, WIDTH, 75), fill="#14314b")
            draw.text((32, 16), "BlueBoat / Gazebo — траектория судна",
                      font=title_font, fill="white")
            draw.text((34, 79), "ВИЗУАЛИЗАЦИЯ ТЕЛЕМЕТРИИ • НЕ ВИДЕО С КАМЕРЫ",
                      font=small_font, fill="#b14435")
            draw.rectangle(PLOT, fill="white", outline="#93a4b5", width=2)
            for k in range(1, 6):
                gx=PLOT[0]+(PLOT[2]-PLOT[0])*k//6
                gy=PLOT[1]+(PLOT[3]-PLOT[1])*k//6
                draw.line((gx,PLOT[1],gx,PLOT[3]),fill="#e7edf2")
                draw.line((PLOT[0],gy,PLOT[2],gy),fill="#e7edf2")
            draw.text((PLOT[0]+12,PLOT[1]+10),
                      "Координаты Gazebo X/Y (метры)",font=small_font,fill="#576879")

            # A pale complete trace gives context; coloured segments only
            # describe positions already observed at this video time.
            if len(coords)>1:
                draw.line(coords,fill="#d4dee7",width=3,joint="curve")
            if end>1:
                before = end if hold_index is None else min(end,hold_index+1)
                if before>1:
                    draw.line(coords[:before],fill="#0b79a2",width=5,joint="curve")
                if hold_index is not None and end>hold_index+1:
                    draw.line(coords[hold_index:end],fill="#dc7c2b",width=5,
                              joint="curve")
            draw.ellipse((coords[0][0]-7,coords[0][1]-7,
                          coords[0][0]+7,coords[0][1]+7),fill="#259a68")
            if hold_index is not None:
                hx,hy=coords[hold_index]
                draw.ellipse((hx-7,hy-7,hx+7,hy+7),fill="#ca4646")
            cx,cy=coords[end-1]
            draw.ellipse((cx-9,cy-9,cx+9,cy+9),fill="#14314b",outline="white",
                         width=3)

            draw.rounded_rectangle((815, 112, 1055, 565), radius=12,
                                   fill="white", outline="#d9e1e8")
            onboard=bool(mission.get("onboard_without_shore"))
            label="Бортовая система" if onboard else "Береговая коррекция"
            phase=("HOLD" if is_hold and current>=hold_at else "AUTO / подготовка")
            entries=[
                ("Режим",phase),
                ("Навигация",label),
                ("Время модели",f"{current-t[0]:.1f} с"),
                ("Смещение от начала",f"{math.hypot(xs[end-1]-xs[0],ys[end-1]-ys[0]):.2f} м"),
                ("Пройдено образцов",f"{end} / {len(truth)}"),
            ]
            if is_hold and current>=hold_at:
                drift=math.hypot(xs[end-1]-xs[hold_index],
                                 ys[end-1]-ys[hold_index])
                entries.append(("Снос после HOLD",f"{drift:.2f} м"))
            for j,(label,value) in enumerate(entries):
                y=139+j*67
                draw.text((835,y),label,font=small_font,fill="#5f7080")
                draw.text((835,y+21),value,font=body_font,fill="#14314b")
            draw.text((55,584),
                      "Источник координат: /model/blueboat/odometry (Gazebo)",
                      font=small_font,fill="#566e80")
            draw.text((55,608),
                      f"Подтверждено: {mode}. Временная шкала ускорена; это не натурные испытания.",
                      font=small_font,fill="#566e80")
            image.save(root/f"frame_{i:04d}.png",format="PNG")

        command=["ffmpeg","-hide_banner","-loglevel","error","-nostdin", "-y",
                 "-framerate",str(fps),"-i",str(root/"frame_%04d.png"),
                 "-c:v","libx264","-crf","25","-preset","veryfast",
                 "-pix_fmt","yuv420p","-movflags","+faststart",str(output)]
        try:
            subprocess.run(command,check=True,capture_output=True,text=True,timeout=150)
        except subprocess.CalledProcessError as exc:
            raise RuntimeError("video encoding failed: "+exc.stderr[-1000:]) from exc
    if not output.is_file() or output.stat().st_size<3000:
        raise RuntimeError("no decodable MP4 was produced")
    return {"file":str(output),"frames":frames,"fps":fps,
            "duration_s":round(frames/fps,2),"source":"verified Gazebo BlueBoat odometry",
            "mode":mode,"not_a_camera_recording":True}


if __name__=="__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("--truth",required=True)
    parser.add_argument("--mission",required=True)
    parser.add_argument("--verification",required=True)
    parser.add_argument("--output",required=True)
    parser.add_argument("--frames",type=int,default=240)
    parser.add_argument("--fps",type=int,default=12)
    args=parser.parse_args()
    try:
        print("BLUEBOAT_VIDEO_READY "+json.dumps(render(
            args.truth,args.mission,args.verification,args.output,
            args.frames,args.fps),ensure_ascii=False),flush=True)
    except (OSError,ValueError,RuntimeError,ImportError) as exc:
        sys.exit("FAIL: "+str(exc))
