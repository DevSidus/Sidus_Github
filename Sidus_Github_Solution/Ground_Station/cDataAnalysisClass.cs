using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace Ground_Station
{
    class cDataAnalysisClass
    {
        public structMsgUdpR01 data;
        public Bitmap graphBitmap;
        public List<GraphDataClass> graphList;

        public void init(ListView lv, Panel pnlGraph)
        {
            lv.DoubleBuffered(true);

            foreach (var prop in data.GetType().GetProperties())
            {
                ListViewItem lvi = new ListViewItem(prop.Name);
                lvi.SubItems.Add(prop.GetValue(data, null).ToString());
                lv.Items.Add(lvi);
            }


            graphBitmap = new Bitmap(pnlGraph.Width, pnlGraph.Height);
            graphList = new List<GraphDataClass>();
        }
        public void update(ListView lv, Panel pnlGraph, Color backColor)
        {

            foreach (var prop in data.GetType().GetProperties())
            {
                ListViewItem lvi = lv.FindItemWithText(prop.Name);
                if (lvi != null)
                {

                        lvi.SubItems[1].Text = string.Format("{0,8:0.00}", Convert.ToDouble(prop.GetValue(data, null).ToString()));

                }
            }

            foreach (GraphDataClass graph in graphList)
            {
                foreach (var prop in data.GetType().GetProperties().Where(x => x.Name == graph.name))
                {
                    string str = prop.GetValue(data, null).ToString();
                    graph.insertData(Convert.ToDouble(prop.GetValue(data, null).ToString()));
                }
            }
            
            Graphics g = Graphics.FromImage(graphBitmap);
            Pen p = new Pen(Color.White);
            g.Clear(backColor);
            g.DrawRectangle(p, new Rectangle(new Point(0, 0), new Size(new Point(pnlGraph.Width - 1, pnlGraph.Height - 1))));
            foreach (GraphDataClass graph in graphList)
            {
                graph.Draw(ref g);
                SolidBrush myBrush = new SolidBrush(graph.penColor);
                g.DrawString(graph.name, new Font("Calibri", 8), myBrush, new Point(32, 16 + 20 * graphList.IndexOf(graph)));
                g.FillRectangle(myBrush, new Rectangle(new Point(16, 16 + 20 * graphList.IndexOf(graph)), new Size(16, 16)));

            }
            Graphics graphPanel = pnlGraph.CreateGraphics();
            graphPanel.DrawImage(graphBitmap, new PointF(0.0f, 0.0f));
            

        }

        public void insertGraph(ListView lv, Panel pnlGraph, int hOffset, int vOffset, double hScale, double vScale, Color lineColor)
        {

            for (int i = 0; i < lv.CheckedItems.Count; i++)
            {
                if (graphList.Find(x => x.name == lv.CheckedItems[i].Text) == null)
                {
                    GraphDataClass graphToAdd = new GraphDataClass(pnlGraph.Size, lv.CheckedItems[i].Text);

                    graphToAdd.hOffset = hOffset;
                    graphToAdd.vOffset = vOffset;
                    graphToAdd.hScale = hScale;
                    graphToAdd.vScale = vScale;
                    graphToAdd.penColor = lineColor;

                    graphList.Add(graphToAdd);
                }
            }
        }

        public void removeGraph(ListView lv)
        {

            for (int i = 0; i < lv.CheckedItems.Count; i++)
            {
                foreach (GraphDataClass graph in graphList.Where(x => x.name == lv.CheckedItems[i].Text))
                {
                    graphList.Remove(graph);
                    break;
                }

            }
        }


        public void updateParametersOfSelectedItems(ListView lv, int hOffset, int vOffset, double hScale, double vScale)
        {
            for (int i = 0; i < lv.CheckedItems.Count; i++)
            {
                foreach (GraphDataClass graph in graphList.Where(x => x.name == lv.CheckedItems[i].Text))
                {
                    graph.hOffset = hOffset;
                    graph.vOffset = vOffset;
                    graph.hScale = hScale;
                    graph.vScale = vScale;
                    break;
                }
            }

        }
         
    }
    class GraphDataClass
    {
        public string name;
        public int dataSize;
        public List<double> dataList;
        public List<Point> pointList;

        public Color penColor;
        public Size drawAreaSize;

        public int hOffset { get; set; }
        public double hScale;
        public int vOffset { get; set; }
        public double vScale;

        public GraphDataClass(Size _drawAreaSize, string _name)
        {
            name = _name;
            drawAreaSize = _drawAreaSize;
            dataList = new List<double>();
            pointList = new List<Point>();
            penColor = Color.Black;

            hOffset = 0;
            vOffset = 0;
            hScale = 1.0;
            vScale = 1.0;

            dataSize = 1000;


            for (int i = 0; i < dataSize; i++)
            {
                dataList.Insert(0, 0);
            }

            pointList.Clear();
            for (int i = 0; i < dataList.Count; i++)
            {
                pointList.Add(new Point(drawAreaSize.Width - (int)(i * hScale), drawAreaSize.Height / 2 - (int)(0 * vScale)));
            }

        }

        public void insertData(double newData)
        {
            dataList.Insert(0, newData);
            dataList.RemoveAt(dataSize);

            pointList.Clear();
            for (int i = 0; i < dataList.Count; i++)
            {
                pointList.Add(new Point(drawAreaSize.Width - (int)(i * hScale) + hOffset, drawAreaSize.Height / 2 - (int)((dataList.ElementAt(i) - vOffset) * vScale)));
            }
        }

        public void Draw(ref Graphics g)
        {
            try
            {
                g.DrawLines(new Pen(penColor), pointList.ToArray());
            }
            catch
            {

            }
        }

    }
}
