// Open Space Syntax Engine (SSE) - Grasshopper C# Component
// Enforces primitive array structures (Point3d[], double[]) and Parallel.For multi-threading
// Author: Napat (Tan) Phasundhiae

using System;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Threading.Tasks;
using Rhino.Geometry;

public class OSSE_SpaceSyntaxSolver
{
    public struct Segment
    {
        public int Id;
        public Point3d P0;
        public Point3d P1;
        public double Length;
        public double Bearing;
    }

    public static void SolveSegmentSyntax(
        List<Curve> inputCurves,
        double metricRadius,
        out List<double> outChoice,
        out List<double> outIntegration,
        out List<double> outNACH,
        out List<double> outNAIN)
    {
        int n = inputCurves.Count;
        Segment[] segments = new Segment[n];

        // 1. Primitive Array Extraction
        for (int i = 0; i < n; i++)
        {
            Curve crv = inputCurves[i];
            Point3d p0 = crv.PointAtStart;
            Point3d p1 = crv.PointAtEnd;
            double dx = p1.X - p0.X;
            double dy = p1.Y - p0.Y;
            double len = Math.Sqrt(dx * dx + dy * dy);
            double bearing = Math.Atan2(dx, dy);

            segments[i] = new Segment
            {
                Id = i,
                P0 = p0,
                P1 = p1,
                Length = len,
                Bearing = bearing
            };
        }

        // 2. Build Angular Dual Adjacency Lists & Fast Metric Brandes Dijkstra
        List<int>[] adjNodes = new List<int>[n];
        List<double>[] adjTurnCost = new List<double>[n];
        List<double>[] adjMetricCost = new List<double>[n];
        for (int i = 0; i < n; i++)
        {
            adjNodes[i] = new List<int>();
            adjTurnCost[i] = new List<double>();
            adjMetricCost[i] = new List<double>();
        }

        const double tol = 0.05;
        for (int i = 0; i < n; i++)
        {
            for (int j = i + 1; j < n; j++)
            {
                bool connected =
                    segments[i].P0.DistanceTo(segments[j].P0) <= tol ||
                    segments[i].P0.DistanceTo(segments[j].P1) <= tol ||
                    segments[i].P1.DistanceTo(segments[j].P0) <= tol ||
                    segments[i].P1.DistanceTo(segments[j].P1) <= tol;
                if (!connected) continue;

                double diff = Math.Abs(segments[i].Bearing - segments[j].Bearing) % (2.0 * Math.PI);
                if (diff > Math.PI) diff = 2.0 * Math.PI - diff;
                double turnWeight = Math.Max(Math.Abs(diff - Math.PI) / (Math.PI * 0.5), 1e-5);
                double metricStep = 0.5 * (segments[i].Length + segments[j].Length);

                adjNodes[i].Add(j);
                adjTurnCost[i].Add(turnWeight);
                adjMetricCost[i].Add(metricStep);

                adjNodes[j].Add(i);
                adjTurnCost[j].Add(turnWeight);
                adjMetricCost[j].Add(metricStep);
            }
        }

        double[][] threadChoice = new double[n][];
        double[] rawChoice = new double[n];
        double[] rawIntegration = new double[n];

        // Multi-threaded parallel Brandes traversal across segments
        Parallel.For(0, n, (int s) =>
        {
            double[] dAng = new double[n];
            double[] dMet = new double[n];
            double[] sigma = new double[n];
            bool[] visited = new bool[n];
            List<int>[] preds = new List<int>[n];
            int[] stack = new int[n];
            int stackTop = 0;

            for (int i = 0; i < n; i++)
            {
                dAng[i] = double.MaxValue;
                dMet[i] = double.MaxValue;
                preds[i] = new List<int>();
            }
            dAng[s] = 0.0;
            dMet[s] = 0.0;
            sigma[s] = 1.0;

            for (int step = 0; step < n; step++)
            {
                int u = -1;
                double bestAng = double.MaxValue;
                for (int i = 0; i < n; i++)
                {
                    if (!visited[i] && dAng[i] < bestAng)
                    {
                        bestAng = dAng[i];
                        u = i;
                    }
                }
                if (u == -1) break;
                visited[u] = true;
                stack[stackTop++] = u;

                List<int> neighbors = adjNodes[u];
                List<double> turns = adjTurnCost[u];
                List<double> metrics = adjMetricCost[u];
                for (int k = 0; k < neighbors.Count; k++)
                {
                    int v = neighbors[k];
                    double newMet = dMet[u] + metrics[k];
                    if (metricRadius > 0.0 && newMet > metricRadius) continue;

                    double newAng = dAng[u] + turns[k];
                    if (newAng < dAng[v] - 1e-7)
                    {
                        dAng[v] = newAng;
                        dMet[v] = newMet;
                        sigma[v] = sigma[u];
                        preds[v].Clear();
                        preds[v].Add(u);
                    }
                    else if (Math.Abs(newAng - dAng[v]) <= 1e-7)
                    {
                        sigma[v] += sigma[u];
                        preds[v].Add(u);
                    }
                }
            }

            double[] delta = new double[n];
            double[] localChoice = new double[n];
            double sumAng = 0.0;
            int reachable = 0;

            while (stackTop > 0)
            {
                int w = stack[--stackTop];
                if (w != s && dAng[w] < double.MaxValue)
                {
                    sumAng += dAng[w];
                    reachable++;
                }
                foreach (int v in preds[w])
                {
                    if (sigma[w] > 0.0)
                    {
                        delta[v] += (sigma[v] / sigma[w]) * (1.0 + delta[w]);
                    }
                }
                if (w != s)
                {
                    localChoice[w] = delta[w];
                }
            }

            threadChoice[s] = localChoice;
            rawIntegration[s] = reachable > 0 ? (double)reachable / Math.Max(sumAng, 1e-4) : 0.0;
        });

        for (int s = 0; s < n; s++)
        {
            if (threadChoice[s] == null) continue;
            for (int i = 0; i < n; i++)
            {
                rawChoice[i] += threadChoice[s][i];
            }
        }

        // 3. Logarithmic Normalization
        outChoice = new List<double>(n);
        outIntegration = new List<double>(n);
        outNACH = new List<double>(n);
        outNAIN = new List<double>(n);

        double maxC = 1.0;
        double maxI = 1.0;
        for (int i = 0; i < n; i++)
        {
            if (rawChoice[i] > maxC) maxC = rawChoice[i];
            if (rawIntegration[i] > maxI) maxI = rawIntegration[i];
        }

        double logMaxC = Math.Log(maxC + 1.0);
        double logMaxI = Math.Log(maxI + 1.0);

        for (int i = 0; i < n; i++)
        {
            outChoice.Add(rawChoice[i]);
            outIntegration.Add(rawIntegration[i]);

            double normC = Math.Log(rawChoice[i] + 1.0) / logMaxC;
            double normI = Math.Log(rawIntegration[i] + 1.0) / logMaxI;

            outNACH.Add(0.55 + 1.25 * normC);
            outNAIN.Add(0.60 + 1.15 * normI);
        }
    }
}
