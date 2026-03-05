package org.example.visualizer;

import org.example.visualizer.spline.CubicSplineSegment;
import org.example.visualizer.spline.Waypoint;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * Simple test harness that generates random waypoints, builds CubicSplineSegment objects using TeamCode classes,
 * samples them and displays the resulting path in a Swing window.
 */
public class VisualizerMain {
    public static void main(String[] args) {
        // Determine seed: if a numeric argument is provided, use it; otherwise use a simple time-based seed that changes between runs
        final long seed;
        if (args != null && args.length > 0) {
            long parsed;
            try {
                parsed = Long.parseLong(args[0]);
            } catch (NumberFormatException ignored) {
                // If not a number, fall back to a nanosecond time-based seed to ensure variability between runs
                parsed = System.nanoTime();
            }
            seed = parsed;
        } else {
            // Use nanoTime so consecutive runs (even within the same millisecond) get different seeds
            seed = System.nanoTime();
        }

        final long chosenSeed = seed;
        // Print the chosen seed to stdout so runs are reproducible when needed
        System.out.println("Visualizer seed: " + chosenSeed);

        SwingUtilities.invokeLater(() -> {
            try {
                new VisualizerMain().start(chosenSeed);
            } catch (Exception e) {
                e.printStackTrace();
            }
        });
    }

    private void start(long randomSeed) {
        // Config
        final int waypointCount = 6; // number of meaningful waypoints (not counting duplicated ends)
        final double coordRange = 48.0; // 48 units (e.g., inches or cm) square
        final double robotTopSpeed = 12.0; // arbitrary units per second

        List<Waypoint> waypoints = generateRandomWaypoints(waypointCount, coordRange, randomSeed);
        List<SegmentInfo> segments = buildSegmentsFromWaypoints(waypoints, robotTopSpeed);
        List<Point2D.Double> samples = sampleSegments(segments, 200);
        List<Marker> markers = computeMarkers(segments, 0.5);

        JFrame frame = new JFrame("Random Spline Visualizer (seed=" + randomSeed + ")");
        frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
        SplinePanel panel = new SplinePanel(samples, waypoints, markers);
        frame.setContentPane(panel);
        frame.setSize(800, 800);
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);

        // Add key binding: press SPACE to re-randomize the spline
        Runnable regenerate = () -> {
            long newSeed = System.nanoTime();
            List<Waypoint> newWaypoints = generateRandomWaypoints(waypointCount, coordRange, newSeed);
            List<SegmentInfo> newSegments = buildSegmentsFromWaypoints(newWaypoints, robotTopSpeed);
            List<Point2D.Double> newSamples = sampleSegments(newSegments, 200);
            List<Marker> newMarkers = computeMarkers(newSegments, 0.5);
            SwingUtilities.invokeLater(() -> {
                panel.updateData(newSamples, newWaypoints, newMarkers);
                frame.setTitle("Random Spline Visualizer (seed=" + newSeed + ")");
                System.out.println("Visualizer seed: " + newSeed);
            });
        };

        InputMap im = panel.getInputMap(JComponent.WHEN_IN_FOCUSED_WINDOW);
        ActionMap am = panel.getActionMap();
        im.put(KeyStroke.getKeyStroke("SPACE"), "reRandomize");
        am.put("reRandomize", new AbstractAction() {
            @Override
            public void actionPerformed(ActionEvent e) {
                regenerate.run();
            }
        });

        frame.addWindowListener(new WindowAdapter() {
            @Override
            public void windowClosing(WindowEvent e) {
                System.exit(0);
            }
        });
    }

    private List<Waypoint> generateRandomWaypoints(int count, double range, long seed) {
        Random rnd = new Random(seed);
        List<Waypoint> pts = new ArrayList<>();
        for (int i = 0; i < count; i++) {
            double x = (rnd.nextDouble() - 0.5) * range;
            double y = (rnd.nextDouble() - 0.5) * range;
            double angle = (rnd.nextDouble() - 0.5) * Math.PI * 2; // -pi..pi
            double speed = 0.1 + rnd.nextDouble() * 0.9; // 0.1..1.0
            boolean hold = false;
            pts.add(new Waypoint(x, y, angle, speed, hold));
        }
        return pts;
    }

    private List<SegmentInfo> buildSegmentsFromWaypoints(List<Waypoint> pts, double robotTopSpeed) {
        List<SegmentInfo> segments = new ArrayList<>();
        if (pts.size() < 2) return segments;

        double startTime = 0.0;
        for (int i = 0; i < pts.size() - 1; i++) {
            Waypoint last = (i - 1 >= 0) ? pts.get(i - 1) : pts.get(i);
            Waypoint start = pts.get(i);
            Waypoint end = pts.get(i + 1);
            Waypoint next = (i + 2 < pts.size()) ? pts.get(i + 2) : pts.get(i + 1);
            CubicSplineSegment seg = new CubicSplineSegment(last, start, end, next, startTime, robotTopSpeed);
            double endTime = seg.endTime();
            segments.add(new SegmentInfo(seg, startTime, endTime));
            startTime = endTime;
        }
        return segments;
    }

    private List<Point2D.Double> sampleSegments(List<SegmentInfo> segments, int samplesPerSegment) {
        List<Point2D.Double> samples = new ArrayList<>();
        for (SegmentInfo info : segments) {
            double startT = info.startTime;
            double endT = info.endTime;
            // guard
            if (endT <= startT) endT = startT + 1e-3;
            for (int i = 0; i < samplesPerSegment; i++) {
                double u = (double) i / (samplesPerSegment - 1);
                double t = startT + u * (endT - startT);
                double x = info.segment.getX(t);
                double y = info.segment.getY(t);
                samples.add(new Point2D.Double(x, y));
            }
        }
        return samples;
    }

    // Compute markers at a fixed interval (seconds) across the whole trajectory
    private List<Marker> computeMarkers(List<SegmentInfo> segments, double intervalSeconds) {
        List<Marker> markers = new ArrayList<>();
        if (segments.isEmpty()) return markers;
        double totalEnd = segments.get(segments.size() - 1).endTime;
        // iterate from time = 0 up to totalEnd (inclusive)
        for (double t = 0.0; t <= totalEnd + 1e-9; t += intervalSeconds) {
            // find segment containing t
            for (SegmentInfo si : segments) {
                if (t + 1e-12 >= si.startTime && t <= si.endTime + 1e-12) {
                    double x = si.segment.getX(t);
                    double y = si.segment.getY(t);
                    double speed = computeSpeed(si.segment, t, si.startTime, si.endTime);
                    markers.add(new Marker(t, x, y));
                    break;
                }
            }
        }
        return markers;
    }

    // Numerical derivative (central difference) to compute instantaneous speed at time t for a segment.
    private static double computeSpeed(CubicSplineSegment seg, double t, double segStart, double segEnd) {
        // choose epsilon relative to segment duration, but not too small
        double segDur = Math.max(1e-3, segEnd - segStart);
        double eps = Math.min(1e-3, segDur * 1e-3);
        // ensure we have a usable interval; if near edges use one-sided difference
        double tPrev = t - eps;
        double tNext = t + eps;
        if (tPrev < segStart) tPrev = t; // will force forward diff
        if (tNext > segEnd) tNext = t;   // will force backward diff

        double xPrev = seg.getX(tPrev);
        double yPrev = seg.getY(tPrev);
        double xNext = seg.getX(tNext);
        double yNext = seg.getY(tNext);

        double dt = tNext - tPrev;
        if (dt <= 0) return 0.0;
        double dx = xNext - xPrev;
        double dy = yNext - yPrev;
        return Math.hypot(dx, dy) / dt;
    }

    private static class SegmentInfo {
        final CubicSplineSegment segment;
        final double startTime;
        final double endTime;

        SegmentInfo(CubicSplineSegment segment, double startTime, double endTime) {
            this.segment = segment;
            this.startTime = startTime;
            this.endTime = endTime;
        }
    }

    private static class Marker {
        final double time;
        final double x, y;

        Marker(double time, double x, double y) {
            this.time = time;
            this.x = x;
            this.y = y;
        }
    }

    private static class SplinePanel extends JPanel {
        private List<Point2D.Double> samples;
        private List<Waypoint> waypoints;
        private List<Marker> markers;

        SplinePanel(List<Point2D.Double> samples, List<Waypoint> waypoints, List<Marker> markers) {
            this.samples = samples;
            this.waypoints = waypoints;
            this.markers = markers;
            setBackground(Color.WHITE);
            setFocusable(true);
        }

        void updateData(List<Point2D.Double> samples, List<Waypoint> waypoints, List<Marker> markers) {
            this.samples = samples;
            this.waypoints = waypoints;
            this.markers = markers;
            repaint();
        }

        @Override
        protected void paintComponent(Graphics g) {
            super.paintComponent(g);
            if (samples == null || samples.isEmpty()) return;
            Graphics2D g2 = (Graphics2D) g;
            g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

            // Compute bounds (include markers)
            double minx = Double.POSITIVE_INFINITY, miny = Double.POSITIVE_INFINITY, maxx = Double.NEGATIVE_INFINITY, maxy = Double.NEGATIVE_INFINITY;
            for (Point2D.Double p : samples) {
                minx = Math.min(minx, p.x);
                miny = Math.min(miny, p.y);
                maxx = Math.max(maxx, p.x);
                maxy = Math.max(maxy, p.y);
            }
            for (Waypoint w : waypoints) {
                minx = Math.min(minx, w.getX());
                miny = Math.min(miny, w.getY());
                maxx = Math.max(maxx, w.getX());
                maxy = Math.max(maxy, w.getY());
            }
            for (Marker m : markers) {
                minx = Math.min(minx, m.x);
                miny = Math.min(miny, m.y);
                maxx = Math.max(maxx, m.x);
                maxy = Math.max(maxy, m.y);
            }

            double pad = 20;
            double ww = Math.max(1, getWidth() - 2 * pad);
            double hh = Math.max(1, getHeight() - 2 * pad);
            double dx = maxx - minx;
            double dy = maxy - miny;
            double scale = Math.min(ww / (dx == 0 ? 1 : dx), hh / (dy == 0 ? 1 : dy));

            // transform function: world to screen
            double cx = minx + dx / 2.0;
            double cy = miny + dy / 2.0;
            double sx = getWidth() / 2.0;
            double sy = getHeight() / 2.0;

            // Draw samples polyline
            g2.setColor(Color.BLUE);
            Point prev = null;
            for (Point2D.Double p : samples) {
                int px = (int) Math.round(sx + (p.x - cx) * scale);
                int py = (int) Math.round(sy - (p.y - cy) * scale); // invert Y
                if (prev != null) {
                    g2.drawLine(prev.x, prev.y, px, py);
                }
                prev = new Point(px, py);
            }

            // Draw waypoints
            g2.setColor(Color.RED);
            for (Waypoint wpt : waypoints) {
                int px = (int) Math.round(sx + (wpt.getX() - cx) * scale);
                int py = (int) Math.round(sy - (wpt.getY() - cy) * scale);
                g2.fillOval(px - 4, py - 4, 8, 8);
                // draw heading
                double ax = Math.cos(wpt.getAngle());
                double ay = Math.sin(wpt.getAngle());
                int hx = (int) Math.round(px + ax * 12);
                int hy = (int) Math.round(py - ay * 12);
                g2.drawLine(px, py, hx, hy);
            }

            // Draw half-second markers
            g2.setColor(new Color(0, 128, 0)); // dark green
            for (Marker m : markers) {
                int px = (int) Math.round(sx + (m.x - cx) * scale);
                int py = (int) Math.round(sy - (m.y - cy) * scale);
                g2.fillOval(px - 3, py - 3, 6, 6);
                // draw speed label small
                String lbl = String.format("%.2f", m.time);
                g2.setFont(g2.getFont().deriveFont(10f));
                g2.drawString(lbl, px + 5, py - 5);
            }
        }
    }
}