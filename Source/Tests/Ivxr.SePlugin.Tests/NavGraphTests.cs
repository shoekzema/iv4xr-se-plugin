﻿
using System.Collections.Generic;
using System.Linq;
using Iv4xr.SpaceEngineers.WorldModel;
using Iv4xr.SePlugin;
using Iv4xr.SePlugin.Control;
using Iv4xr.SePlugin.Navigation;
using VRageMath;
using Xunit;

namespace Ivxr.SeGameLib.Tests
{
    public class NavGraphTests
    {
        private static IEnumerable<PlainVec3I> TrivialGridGenerator()
        {
            for (var x = 0; x < 3; x++)
            {
                yield return new PlainVec3I(x, 0, 0);
            }
        }
        
        private static CubeGrid CreateTestGrid(IEnumerable<PlainVec3I> gridGenerator)
        {
            var grid = new CubeGrid()
            {
                Blocks = new List<Block>()
            };

            foreach (var gridPosition in gridGenerator)
            {
                var block = new Block
                {
                    GridPosition = gridPosition,
                    Position = new Vector3D(gridPosition.ToVector3I()).ToPlain()
                };
                
                grid.Blocks.Add(block);
            }

            return grid;
        }

        private static FatNavGraph CreateGraph(IEnumerable<PlainVec3I> gridGenerator)
        {
            // We will not call methods that use LowLevelObserver
            var graphEditor = new NavGraphEditor(new LowLevelObserver(new GameSession()));

            var grid = CreateTestGrid(gridGenerator);
            
            return graphEditor.CreateGraph(grid, Vector3D.Zero, Vector3I.Up);
        }
        
        [Fact]
        public void CreatesTrivialGraph()
        {
            var graph = CreateGraph(TrivialGridGenerator());

            Assert.Equal(3, graph.Nodes.Count);
        }

        IEnumerable<PlainVec3I> GenGridPositions(int width, int length)
        {
            for (var x = 1; x <= width; x++)
                for (var z = 1; z <= length; z++)
                    yield return new PlainVec3I(x, 0, z);
        }
        
        [Theory]
        [InlineData(2, 2)]
        [InlineData(1, 5)]
        [InlineData(3, 4)]
        [InlineData(10, 17)]
        public void CreatesRectangularGraph(int width, int length)
        {
            var graph = CreateGraph(GenGridPositions(width, length));
            
            Assert.Equal(width * length, graph.Nodes.Count);

            if (width >= 3 && length >= 3)
            {
                var maxEdges = graph.Nodes.Select(n => n.Neighbours.Count).Max();
                Assert.Equal(4, maxEdges);
            }

            if (width >= 2 && length >= 2)
            {
                var minEdges = graph.Nodes.Select(n => n.Neighbours.Count).Min();
                Assert.Equal(2, minEdges);
            }
        }

        [Theory]
        [InlineData(2, 1)]
        //[InlineData(1, 1)] // Doesn't work because the starting position has an obstacle above it.
        [InlineData(3, 1)]
        [InlineData(2, 2)]
        public void CreatesGraphForGridWithObstacles(int x, int z)
        {
            const int width = 3;
            const int length = 4;
            var fullEdges = CalculateFullEdgeCount(length, width);

            var graph = CreateGraph(GetGridWithObstacle());
            var edges = graph.Nodes.Select(n => n.Neighbours.Count).Sum() / 2;

            Assert.Equal(width*length - 1, graph.Nodes.Count);
            
            if (x == 2 && z == 1)
            {
                Assert.Equal(fullEdges - 3, edges);
            }
            else if (x == 1 && z == 3)
            {
                Assert.Equal(fullEdges - 2, edges);
            }
            else if (x == 2 && z == 2)
            {
                Assert.Equal(fullEdges - 4, edges);
            }
            
            IEnumerable<PlainVec3I> GetGridWithObstacle()
            {
                foreach (var position in GenGridPositions(width, length))
                {
                    // add and obstacle
                    if (position.ToVector3I().Equals(new Vector3I(x, 0, z)))
                        yield return new PlainVec3I(x, 1, z);

                    yield return position;
                }
            }
        }

        private static int CalculateFullEdgeCount(int length, int width)
        {
            var l = length - 2;
            var w = width - 2;
            
            // 2 * corners + 3 * sides + 4 * inner_tiles
            return (2 * 4 + 3 * 2 * (l + w) + 4 * l * w) / 2;
        }

        [Fact]
        public void CreatesGraphForNontrivialGrid()
        {
            const int width = 4;
            const int length = 4;
            
            var graph = CreateGraph(GetGrid());
            var edges = graph.Nodes.Select(n => n.Neighbours.Count).Sum() / 2;

            Assert.Equal(width*length - 5, graph.Nodes.Count);
            Assert.Equal(CalculateFullEdgeCount(length - 2, width) + 3, edges);
            
            IEnumerable<PlainVec3I> GetGrid()
            {
                foreach (var position in GenGridPositions(width, length))
                {
                    // add a hole
                    if (Enumerable.Range(1, 3).Contains(position.X) && position.Y == 0 && position.Z == 3)
                        continue;
                    
                    // add an obstacle
                    if (position.ToVector3I().Equals(new Vector3I(2, 0, 4)))
                        yield return new PlainVec3I(2, 1, 4);
                    
                    yield return position;
                }
            } 
        }

        [Theory]
        [InlineData(1, 5)]
        [InlineData(4, 7)]
        public void ConvertsRectangularGraph(int width, int length)
        {
            var fatGraph = CreateGraph(GenGridPositions(width, length));
            var slimGraph = fatGraph.ToNavGraph();
            
            Assert.Equal(fatGraph.Nodes.Count, slimGraph.Nodes.Count);

            var edges = fatGraph.Nodes.Select(n => n.Neighbours.Count).Sum() / 2;
            Assert.Equal(edges, slimGraph.Edges.Count);
        }

        [Fact]
        public void GuessesWhichSideIsUp()
        {
            Assert.True(NavGraphEditor.GuessWhichSideIsUp(new Vector3D(1.012E-06, 5.182E-05, 0.999))
                        == Vector3I.Backward);
            
            Assert.True(NavGraphEditor.GuessWhichSideIsUp(new Vector3D(-1.012E-06, -5.182E-05, -0.999))
                        == Vector3I.Forward);
            
            Assert.True(NavGraphEditor.GuessWhichSideIsUp(new Vector3D(-0.2, 0.7, 0.1)) == Vector3I.Up);
            
            Assert.True(NavGraphEditor.GuessWhichSideIsUp(new Vector3D(-10.2, 5.7, 0)) == Vector3I.Left);
        }
    }
}
