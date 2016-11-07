using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace Ground_Station
{ 
    public partial class Ground_Station : Form
    {
        //Initial Declaration of Objects
        cUdpSniffer qgsUdp = new cUdpSniffer(cConfig.UDP_PORT_NUM);
        public Ground_Station()
        {
            vfnDeclaration();
        }        
        public void vfnDeclaration()
        {
            //Initialize Form
            InitializeComponent();
            //Subscribe BackgroundWorker bwUdpSniffer
            bwUdpSniffer.WorkerReportsProgress = true;
            bwUdpSniffer.DoWork += new DoWorkEventHandler(bwUdpSniffer_DoWork);
            bwUdpSniffer.ProgressChanged += new ProgressChangedEventHandler(bwUdpSniffer_ProgressChanged);
        }
        private void bwUdpSniffer_ProgressChanged(object sender, ProgressChangedEventArgs e)
        {
            prbMainProgress.Value = e.ProgressPercentage;
        }
        private void bwUdpSniffer_DoWork(object sender, DoWorkEventArgs e)
        {
            while (true)
            {
                if (pnlHeartBeat.BackColor == System.Drawing.Color.OrangeRed)
                {
                    pnlHeartBeat.BackColor = System.Drawing.Color.Green;
                }
                else
                {
                    pnlHeartBeat.BackColor = System.Drawing.Color.OrangeRed;
                }
            }
        }
        private void Ground_Station_Load(object sender, EventArgs e)
        {

            //Bugun tum ekipce skype uzerinden görüntülü görüşme yaptık.
            bwUdpSniffer.RunWorkerAsync();
        }
    }
}
