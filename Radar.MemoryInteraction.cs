using System;
using System.Linq;
using System.Threading.Tasks;
using ExileCore.Shared.Helpers;
using SharpDX.Win32;
using SixLabors.ImageSharp;
using SixLabors.ImageSharp.PixelFormats;
using SixLabors.ImageSharp.Processing;
using SixLabors.ImageSharp.Processing.Processors.Convolution;
using SixLabors.ImageSharp.Processing.Processors.Transforms;
using TravelShortPathFinder.Algorithm.Utils;
using Configuration = SixLabors.ImageSharp.Configuration;
using Vector4 = System.Numerics.Vector4;

namespace Radar;

public partial class Radar
{
    private void GenerateMapTexture()
    {
        var gridHeightData = _heightData;
        var maxX = _areaDimensions.Value.X;
        var maxY = _areaDimensions.Value.Y;
        var configuration = Configuration.Default.Clone();
        configuration.PreferContiguousImageBuffers = true;
        using var image = new Image<Rgba32>(configuration, maxX, maxY);
        var unwalkableMask = Vector4.UnitX + Vector4.UnitW;
        var walkableMask = Vector4.UnitY + Vector4.UnitW;
        if (Settings.Debug.DrawHeightMap)
        {
            var minHeight = gridHeightData.Min(x => x.Min());
            var maxHeight = gridHeightData.Max(x => x.Max());
            image.Mutate(configuration, c => c.ProcessPixelRowsAsVector4((row, i) =>
            {
                for (var x = 0; x < row.Length - 1; x += 2)
                {
                    var cellData = gridHeightData[i.Y][x];
                    for (var x_s = 0; x_s < 2; ++x_s)
                    {
                        row[x + x_s] = new Vector4(0, (cellData - minHeight) / (maxHeight - minHeight), 0, 1);
                    }
                }
            }));
        }
        else
        {
           
            if (Settings.Debug.DisableHeightAdjust)
            {
                Parallel.For(0, maxY, y =>
                {
                    for (var x = 0; x < maxX; x++)
                    {
                        var terrainType = _processedTerrainData[y][x];
                        image[x, y] = new Rgba32(terrainType < 2 ? unwalkableMask : walkableMask);
                    }
                });
            }
            else
            {
                Parallel.For(0, maxY, y =>
                {
                    for (var x = 0; x < maxX; x++)
                    {
                        var cellData = gridHeightData[y][x / 2 * 2];

                        //basically, offset x and y by half the offset z would cause when rendering in 3d
                        var heightOffset = (int)(cellData / GridToWorldMultiplier / 2);
                        var offsetX = x - heightOffset;
                        var offsetY = y - heightOffset;
                        var terrainType = _processedTerrainData[y][x];
                        if (offsetX >= 0 && offsetX < maxX && offsetY >= 0 && offsetY < maxY)
                        {
                            image[offsetX, offsetY] = new Rgba32(terrainType < 2 ? unwalkableMask : walkableMask);
                        }
                    }
                });
            }

            if (!Settings.Debug.SkipNeighborFill)
            {
                Parallel.For(0, maxY, y =>
                {
                    for (var x = 0; x < maxX; x++)
                    {
                        //this fills in the blanks that are left over from the height projection
                        if (image[x, y].ToVector4() == Vector4.Zero)
                        {
                            var countWalkable = 0;
                            var countUnwalkable = 0;
                            for (var xO = -1; xO < 2; xO++)
                            {
                                for (var yO = -1; yO < 2; yO++)
                                {
                                    var xx = x + xO;
                                    var yy = y + yO;
                                    if (xx >= 0 && xx < maxX && yy >= 0 && yy < maxY)
                                    {
                                        var nPixel = image[x + xO, y + yO].ToVector4();
                                        if (nPixel == walkableMask)
                                            countWalkable++;
                                        else if (nPixel == unwalkableMask)
                                            countUnwalkable++;
                                    }
                                }
                            }

                            image[x, y] = new Rgba32(countWalkable > countUnwalkable ? walkableMask : unwalkableMask);
                        }
                    }
                });
            }

            if (!Settings.Debug.SkipEdgeDetector)
            {
                var edgeDetector = new EdgeDetectorProcessor(EdgeDetectorKernel.Laplacian5x5, false)
                   .CreatePixelSpecificProcessor(configuration, image, image.Bounds());
                edgeDetector.Execute();
            }

            //LogMessage($"Generating navGrid");
            //_navGrid = NavGridProvider.CreateGrid(maxX, maxY, p => image[p.X, p.Y].ToVector4().Equals(walkableMask));
            //_explorer = new TravelShortPathFinder.Algorithm.Logic.GraphMapExplorer(_navGrid, new TravelShortPathFinder.Algorithm.Data.Settings(localSelectNearNodeRange:4,segmentationSquareSize: 50, segmentationMinSegmentSize: 300, playerVisibilityRadius: 140, fastSegmentationThroughOnePoint: true));
            //var p = new System.Drawing.Point((int)GameController.Player.GridPosNum.X, (int)GameController.Player.GridPosNum.Y);
            //_explorer.ProcessSegmentation(p);
            //LogMessage($"Processed segmentation ({p})");

            if (!Settings.Debug.SkipRecoloring)
            {
                image.Mutate(configuration, c => c.ProcessPixelRowsAsVector4((row, p) =>
                {
                    for (var x = 0; x < row.Length - 0; x++)
                    {
                        row[x] = row[x] switch
                        {
                            { X: 1 } => Settings.TerrainColor.Value.ToImguiVec4(),
                            { X: 0 } => Vector4.Zero,
                            var s => s
                        };
                    }
                }));
            }
        }

        if (Math.Max(image.Height, image.Width) > Settings.MaximumMapTextureDimension)
        {
            var (newWidth, newHeight) = (image.Width, image.Height);
            if (image.Height > image.Width)
            {
                newWidth = newWidth * Settings.MaximumMapTextureDimension / newHeight;
                newHeight = Settings.MaximumMapTextureDimension;
            }
            else
            {
                newHeight = newHeight * Settings.MaximumMapTextureDimension / newWidth;
                newWidth = Settings.MaximumMapTextureDimension;
            }

            var targetSize = new Size(newWidth, newHeight);
            var resizer = new ResizeProcessor(new ResizeOptions { Size = targetSize }, image.Size())
                .CreatePixelSpecificCloningProcessor(configuration, image, image.Bounds());
            resizer.Execute();
        }

        //unfortunately the library doesn't respect our allocation settings above
        
        using var imageCopy = image.Clone(configuration);

        //image.Mutate(configuration, c => c.ProcessPixelRowsAsVector4((row, p) =>
        //{
        //    for (var x = 0; x < row.Length - 0; x++)
        //    {
        //        row[x] = row[x] switch
        //        {
        //            { X: > 0 } => SharpDX.Color.Black.ToImguiVec4(),
        //            { X: 0 } => SharpDX.Color.White.ToImguiVec4(),
        //            var s => s
        //        };
        //    }
        //}));
        var pos = GameController.Player.GridPosNum;
        image.Mutate(configuration, c => c.ProcessPixelRowsAsVector4((row, p) =>
        {
            for (var x = 0; x < row.Length - 0; x++)
            {
                row[x][0] = 1 - row[x][0];
                row[x][1] = 1 - row[x][1];
                row[x][2] = 1 - row[x][2];

            }
        }));
        image[(int)pos.X, (int)pos.Y] = new Rgba32(150, 150, 150);
        image.ProcessPixelRows(p => {
            for (int rowIndex = 0; rowIndex < p.Height / 2; rowIndex++)
            {
                // Get the row span for the current row and the corresponding bottom row
                var topRow = p.GetRowSpan(rowIndex);
                var bottomRow = p.GetRowSpan(p.Height - rowIndex - 1);
                var tmp = new Span<Rgba32>(topRow.ToArray());
                // Swap the two rows
                bottomRow.CopyTo(topRow);
                tmp.CopyTo(bottomRow);
                
            }

        });
       
        image.SaveAsBmp("test.bmp");
        Graphics.LowLevel.AddOrUpdateTexture(TextureName, imageCopy);
    }
}
