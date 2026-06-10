// Canvas renderer: the VEX field as a brass-framed night chart.
// Field frame: origin at center, +x right, +y up, inches.

import type { World } from '../sim/world.ts';

const FIELD = 144; // inches
const MARGIN = 10; // inches of breathing room around the perimeter

export interface FieldTarget {
  x: number;
  y: number;
}

export class FieldView {
  canvas: HTMLCanvasElement;
  target: FieldTarget | null = null;
  showGhost = true;
  showTrails = true;

  private ctx: CanvasRenderingContext2D;
  private size = 0;

  constructor(canvas: HTMLCanvasElement) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d')!;
    const observer = new ResizeObserver(() => this.resize());
    observer.observe(canvas.parentElement!);
    this.resize();
  }

  private resize(): void {
    const parent = this.canvas.parentElement!;
    const px = Math.min(parent.clientWidth, parent.clientHeight);
    if (px <= 0) return;
    const dpr = window.devicePixelRatio || 1;
    this.size = px;
    this.canvas.width = px * dpr;
    this.canvas.height = px * dpr;
    this.canvas.style.width = `${px}px`;
    this.canvas.style.height = `${px}px`;
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }

  private get scale(): number {
    return this.size / (FIELD + MARGIN * 2);
  }

  /** field inches -> canvas px */
  toPx(x: number, y: number): [number, number] {
    const s = this.scale;
    return [this.size / 2 + x * s, this.size / 2 - y * s];
  }

  /** canvas px -> field inches */
  toField(px: number, py: number): [number, number] {
    const s = this.scale;
    return [(px - this.size / 2) / s, (this.size / 2 - py) / s];
  }

  render(world: World, timeMs: number): void {
    const ctx = this.ctx;
    const s = this.scale;
    ctx.clearRect(0, 0, this.size, this.size);

    this.drawField(ctx, s);
    if (this.showTrails) this.drawTrails(ctx, world);
    if (this.target) this.drawTarget(ctx, this.target, timeMs);

    // odometry's belief, drawn as a ghost under the true robot
    if (this.showGhost) {
      const odom = world.chassis.getPose(true);
      this.drawRobot(ctx, odom.x, odom.y, odom.theta, world, true);
    }
    this.drawRobot(ctx, world.robot.x, world.robot.y, world.robot.theta, world, false);
  }

  private drawField(ctx: CanvasRenderingContext2D, s: number): void {
    const [left, top] = this.toPx(-FIELD / 2, FIELD / 2);
    const sidePx = FIELD * s;

    // foam tiles, subtly alternating like a checkerboard
    for (let i = 0; i < 6; i++) {
      for (let j = 0; j < 6; j++) {
        ctx.fillStyle = (i + j) % 2 === 0 ? '#262012' : '#221c10';
        ctx.fillRect(left + i * 24 * s, top + j * 24 * s, 24 * s, 24 * s);
      }
    }

    // tile seams
    ctx.strokeStyle = 'rgba(202, 160, 74, 0.14)';
    ctx.lineWidth = 1;
    for (let i = 0; i <= 6; i++) {
      ctx.beginPath();
      ctx.moveTo(left + i * 24 * s, top);
      ctx.lineTo(left + i * 24 * s, top + sidePx);
      ctx.moveTo(left, top + i * 24 * s);
      ctx.lineTo(left + sidePx, top + i * 24 * s);
      ctx.stroke();
    }

    // center axes, a navigator's crosshair
    const [cx, cy] = this.toPx(0, 0);
    ctx.strokeStyle = 'rgba(202, 160, 74, 0.30)';
    ctx.setLineDash([2, 6]);
    ctx.beginPath();
    ctx.moveTo(left, cy);
    ctx.lineTo(left + sidePx, cy);
    ctx.moveTo(cx, top);
    ctx.lineTo(cx, top + sidePx);
    ctx.stroke();
    ctx.setLineDash([]);

    // axis labels
    ctx.fillStyle = 'rgba(202, 160, 74, 0.55)';
    ctx.font = `${Math.max(10, 11 * (s / 5))}px 'Cinzel', serif`;
    ctx.textAlign = 'center';
    ctx.fillText('+Y', cx + 12, top + 14);
    ctx.fillText('+X', left + sidePx - 14, cy - 8);

    // brass perimeter with corner rivets
    ctx.strokeStyle = '#8a6420';
    ctx.lineWidth = 5;
    ctx.strokeRect(left, top, sidePx, sidePx);
    ctx.strokeStyle = '#c9972c';
    ctx.lineWidth = 2;
    ctx.strokeRect(left, top, sidePx, sidePx);
    ctx.fillStyle = '#e0b54e';
    for (const [rx, ry] of [
      [left, top], [left + sidePx, top], [left, top + sidePx], [left + sidePx, top + sidePx],
    ]) {
      ctx.beginPath();
      ctx.arc(rx, ry, 3.5, 0, Math.PI * 2);
      ctx.fill();
    }
  }

  private drawTrails(ctx: CanvasRenderingContext2D, world: World): void {
    const drawTrail = (trail: { x: number; y: number }[], style: string, dash: number[]) => {
      if (trail.length < 2) return;
      ctx.strokeStyle = style;
      ctx.lineWidth = 2;
      ctx.setLineDash(dash);
      ctx.beginPath();
      const [x0, y0] = this.toPx(trail[0].x, trail[0].y);
      ctx.moveTo(x0, y0);
      for (let i = 1; i < trail.length; i++) {
        const [x, y] = this.toPx(trail[i].x, trail[i].y);
        ctx.lineTo(x, y);
      }
      ctx.stroke();
      ctx.setLineDash([]);
    };

    // odometry's belief: dashed brass. Ground truth: glowing copper
    drawTrail(world.odomTrail, 'rgba(224, 181, 78, 0.55)', [5, 5]);
    ctx.shadowColor = 'rgba(210, 138, 77, 0.8)';
    ctx.shadowBlur = 6;
    drawTrail(world.trueTrail, '#d28a4d', []);
    ctx.shadowBlur = 0;
  }

  private drawTarget(ctx: CanvasRenderingContext2D, target: FieldTarget, timeMs: number): void {
    const [x, y] = this.toPx(target.x, target.y);
    const pulse = 8 + 3 * Math.sin(timeMs / 280);
    ctx.strokeStyle = '#e0b54e';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(x, y, pulse, 0, Math.PI * 2);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(x - 14, y);
    ctx.lineTo(x + 14, y);
    ctx.moveTo(x, y - 14);
    ctx.lineTo(x, y + 14);
    ctx.stroke();
  }

  private drawRobot(
    ctx: CanvasRenderingContext2D,
    x: number, y: number, thetaMath: number,
    world: World, ghost: boolean,
  ): void {
    const s = this.scale;
    const w = world.config.robotWidth * s;
    const l = world.config.robotLength * s;
    const [px, py] = this.toPx(x, y);

    ctx.save();
    ctx.translate(px, py);
    // canvas y is flipped, so math CCW becomes canvas CW
    ctx.rotate(-thetaMath);

    if (ghost) {
      // odometry ghost: hollow brass outline
      ctx.globalAlpha = 0.55;
      ctx.strokeStyle = '#e0b54e';
      ctx.lineWidth = 1.5;
      ctx.setLineDash([4, 4]);
      ctx.strokeRect(-l / 2, -w / 2, l, w);
      ctx.beginPath();
      ctx.moveTo(0, 0);
      ctx.lineTo(l / 2, 0);
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.restore();
      return;
    }

    // hull
    const grad = ctx.createLinearGradient(0, -w / 2, 0, w / 2);
    grad.addColorStop(0, '#7a5530');
    grad.addColorStop(1, '#3c2a16');
    ctx.fillStyle = grad;
    ctx.strokeStyle = '#c9972c';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.roundRect(-l / 2, -w / 2, l, w, 3);
    ctx.fill();
    ctx.stroke();

    // wheels
    ctx.fillStyle = '#17120a';
    const wheelL = l * 0.28;
    const wheelW = w * 0.14;
    for (const side of [-1, 1]) {
      for (const fore of [-1, 0, 1]) {
        ctx.beginPath();
        ctx.roundRect(
          fore * l * 0.31 - wheelL / 2,
          side * (w / 2 - wheelW * 0.6) - wheelW / 2,
          wheelL, wheelW, 2,
        );
        ctx.fill();
      }
    }

    // bow arrow, pennant red like the logo's flag
    ctx.fillStyle = '#8f2f25';
    ctx.strokeStyle = '#e0b54e';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(l / 2 - 2, 0);
    ctx.lineTo(l * 0.14, -w * 0.2);
    ctx.lineTo(l * 0.14, w * 0.2);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    // brass hub rivet
    ctx.fillStyle = '#e0b54e';
    ctx.beginPath();
    ctx.arc(0, 0, 2.5, 0, Math.PI * 2);
    ctx.fill();

    ctx.restore();
  }
}
