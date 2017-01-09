using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace Ground_Station
{ 
    public partial class Ground_Station : Form
    {
        //Initial Declaration of Objects
        cUdpSniffer qgsUdp = new cUdpSniffer(cConfig.UDP_PORT_NUM);
        cMsgUdpT01 MsgUdpT01 = new cMsgUdpT01();
        cMsgUdpR01 MsgUdpR01 = new cMsgUdpR01();

        cDataAnalysisClass DataAnalysisObj = new cDataAnalysisClass();
        cDataTxDisplayClass DataTxDisplayObj = new cDataTxDisplayClass();
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
            //bwUdpSniffer.ProgressChanged += new ProgressChangedEventHandler(bwUdpSniffer_ProgressChanged);
            

        }
        private void bwUdpSniffer_ProgressChanged(object sender, ProgressChangedEventArgs e)
        {
            //prbMainProgress.Value = e.ProgressPercentage;
        }
        private void bwUdpSniffer_DoWork(object sender, DoWorkEventArgs e)
        {
            
            while (true)
            {

                checkUdpClientStatus();
                /*
                if (pnlHeartBeat.BackColor == System.Drawing.Color.OrangeRed)
                {
                    pnlHeartBeat.BackColor = System.Drawing.Color.Green;
                }
                else
                {
                    pnlHeartBeat.BackColor = System.Drawing.Color.OrangeRed;
                }

                    */
                System.Threading.Thread.Sleep(20);
                          
            }
        }
        private void Ground_Station_Load(object sender, EventArgs e)
        {
            //Bugun tum ekipce skype uzerinden görüntülü görüşme yaptık.

            MsgUdpT01.message.pidAnglePitchKd = 120;
            DataAnalysisObj.init(lvDataAnalysis, pnlDataAnalysis);
            DataTxDisplayObj.init(lvDataTx, MsgUdpT01.message);


            bwUdpSniffer.RunWorkerAsync();
            timerDisplayRefresh.Enabled = true;

            ssMainLabel1.Text = "IP:" + qgsUdp.GetLocalIPv4();
            ssMainLabel2.Text = "Port#:" + qgsUdp.port.ToString();
        }
        

        private void checkUdpClientStatus()
        {
            if (qgsUdp.clientConnected)
            {
                ssMainLabel3.Text = "Connected";
                ssMainLabel4.Text = "Client IP:" + qgsUdp.GetRemoteIP();
            }
            else
            {
                ssMainLabel3.Text = "Not Connected";
                ssMainLabel4.Text = "Client IP:" + "---";
            }
        }
               

        private void timerDisplayRefresh_Tick(object sender, EventArgs e)
        {
            updateMessagesForDataAnalysis();
            DataAnalysisObj.update(lvDataAnalysis, pnlDataAnalysis, tbGraphBackColor.BackColor);
            DataTxDisplayObj.update(lvDataTx, MsgUdpT01.message);
        }

        private void updateMessagesForDataAnalysis()
        {
            DataAnalysisObj.data.R1_baroAlt = MsgUdpR01.message.coWorkerTxPacket.baroAlt;
            DataAnalysisObj.data.R1_baroTemp = MsgUdpR01.message.coWorkerTxPacket.baroTemp;
            DataAnalysisObj.data.R1_batteryVoltageInBits = MsgUdpR01.message.coWorkerTxPacket.batteryVoltageInBits;
            DataAnalysisObj.data.R1_compassHdg = MsgUdpR01.message.coWorkerTxPacket.compassHdg;
            DataAnalysisObj.data.R1_mpuAccRealX = MsgUdpR01.message.coWorkerTxPacket.mpuAccRealX;
            DataAnalysisObj.data.R1_mpuAccRealY = MsgUdpR01.message.coWorkerTxPacket.mpuAccRealY;
            DataAnalysisObj.data.R1_mpuAccRealZ = MsgUdpR01.message.coWorkerTxPacket.mpuAccRealZ;
            DataAnalysisObj.data.R1_mpuAccX = MsgUdpR01.message.coWorkerTxPacket.mpuAccX;
            DataAnalysisObj.data.R1_mpuAccY = MsgUdpR01.message.coWorkerTxPacket.mpuAccY;
            DataAnalysisObj.data.R1_mpuAccZ = MsgUdpR01.message.coWorkerTxPacket.mpuAccZ;
            DataAnalysisObj.data.R1_mpuGyroX = MsgUdpR01.message.coWorkerTxPacket.mpuGyroX;
            DataAnalysisObj.data.R1_mpuGyroY = MsgUdpR01.message.coWorkerTxPacket.mpuGyroY;
            DataAnalysisObj.data.R1_mpuGyroZ = MsgUdpR01.message.coWorkerTxPacket.mpuGyroZ;
            DataAnalysisObj.data.R1_mpuPitch = MsgUdpR01.message.coWorkerTxPacket.mpuPitch;
            DataAnalysisObj.data.R1_mpuRoll = MsgUdpR01.message.coWorkerTxPacket.mpuRoll;
            DataAnalysisObj.data.R1_mpuYaw = MsgUdpR01.message.coWorkerTxPacket.mpuYaw;
            DataAnalysisObj.data.R2_pidAnglePitchKd = MsgUdpR01.message.serialR01RelayPacket.pidAnglePitchKd;
            DataAnalysisObj.data.R2_pidAnglePitchKi = MsgUdpR01.message.serialR01RelayPacket.pidAnglePitchKi;
            DataAnalysisObj.data.R2_pidAnglePitchKp = MsgUdpR01.message.serialR01RelayPacket.pidAnglePitchKp;
            DataAnalysisObj.data.R2_pidAngleRollKd = MsgUdpR01.message.serialR01RelayPacket.pidAngleRollKd;
            DataAnalysisObj.data.R2_pidAngleRollKi = MsgUdpR01.message.serialR01RelayPacket.pidAngleRollKi;
            DataAnalysisObj.data.R2_pidAngleRollKp = MsgUdpR01.message.serialR01RelayPacket.pidAngleRollKp;
            DataAnalysisObj.data.R2_pidAngleYawKd = MsgUdpR01.message.serialR01RelayPacket.pidAngleYawKd;
            DataAnalysisObj.data.R2_pidAngleYawKi = MsgUdpR01.message.serialR01RelayPacket.pidAngleYawKi;
            DataAnalysisObj.data.R2_pidAngleYawKp = MsgUdpR01.message.serialR01RelayPacket.pidAngleYawKp;
            DataAnalysisObj.data.R2_pidRatePitchKd = MsgUdpR01.message.serialR01RelayPacket.pidRatePitchKd;
            DataAnalysisObj.data.R2_pidRatePitchKi = MsgUdpR01.message.serialR01RelayPacket.pidRatePitchKi;
            DataAnalysisObj.data.R2_pidRatePitchKp = MsgUdpR01.message.serialR01RelayPacket.pidRatePitchKp;
            DataAnalysisObj.data.R2_pidRateRollKd = MsgUdpR01.message.serialR01RelayPacket.pidRateRollKd;
            DataAnalysisObj.data.R2_pidRateRollKi = MsgUdpR01.message.serialR01RelayPacket.pidRateRollKi;
            DataAnalysisObj.data.R2_pidRateRollKp = MsgUdpR01.message.serialR01RelayPacket.pidRateRollKp;
            DataAnalysisObj.data.R2_pidRateYawKd = MsgUdpR01.message.serialR01RelayPacket.pidRateYawKd;
            DataAnalysisObj.data.R2_pidRateYawKi = MsgUdpR01.message.serialR01RelayPacket.pidRateYawKi;
            DataAnalysisObj.data.R2_pidRateYawKp = MsgUdpR01.message.serialR01RelayPacket.pidRateYawKp;
            DataAnalysisObj.data.R2_rxPitch = MsgUdpR01.message.serialR01RelayPacket.rxPitch;
            DataAnalysisObj.data.R2_rxRoll = MsgUdpR01.message.serialR01RelayPacket.rxRoll;
            DataAnalysisObj.data.R2_rxThrottle = MsgUdpR01.message.serialR01RelayPacket.rxThrottle;
            DataAnalysisObj.data.R2_rxYaw = MsgUdpR01.message.serialR01RelayPacket.rxYaw;

        }

        private void button_GraphSelectBackColor_Click(object sender, EventArgs e)
        {
            colorDialog_Graph.ShowDialog();
            tbGraphBackColor.BackColor = colorDialog_Graph.Color;
        }

        private void button_GraphSelectColor_Click(object sender, EventArgs e)
        {
            colorDialog_Graph.ShowDialog();
            tbGraphLineColor.BackColor = colorDialog_Graph.Color;
        }

        private void button_GraphSetDefault_Click(object sender, EventArgs e)
        {
            trackBar_hOffset.Value = 0;
            trackBar_hOffset_Scroll(this, null);

            trackBar_vOffset.Value = 0;
            trackBar_vOffset_Scroll(this, null);

            numericUpDown_hScale.Value = (decimal)1.0;
            numericUpDown_hScale_ValueChanged(this, null);

            numericUpDown_vScale.Value = (decimal)1.0;
            numericUpDown_vScale_ValueChanged(this, null);
        }

        private void trackBar_vOffset_Scroll(object sender, EventArgs e)
        {
            DataAnalysisObj.updateParametersOfSelectedItems(lvDataAnalysis, trackBar_hOffset.Value, trackBar_vOffset.Value, (double)numericUpDown_hScale.Value, (double)numericUpDown_vScale.Value);

            textBox_trackBarGraphVer.Text = trackBar_vOffset.Value.ToString();
        }

        private void trackBar_hOffset_Scroll(object sender, EventArgs e)
        {
            DataAnalysisObj.updateParametersOfSelectedItems(lvDataAnalysis, trackBar_hOffset.Value, trackBar_vOffset.Value, (double)numericUpDown_hScale.Value, (double)numericUpDown_vScale.Value);

            textBox_trackBarGraphHor.Text = trackBar_hOffset.Value.ToString();
        }

        private void numericUpDown_hScale_ValueChanged(object sender, EventArgs e)
        {
            DataAnalysisObj.updateParametersOfSelectedItems(lvDataAnalysis, trackBar_hOffset.Value, trackBar_vOffset.Value, (double)numericUpDown_hScale.Value, (double)numericUpDown_vScale.Value);

        }

        private void numericUpDown_vScale_ValueChanged(object sender, EventArgs e)
        {
            DataAnalysisObj.updateParametersOfSelectedItems(lvDataAnalysis, trackBar_hOffset.Value, trackBar_vOffset.Value, (double)numericUpDown_hScale.Value, (double)numericUpDown_vScale.Value);
        }

        private void button_InsertGraph_Click(object sender, EventArgs e)
        {
            DataAnalysisObj.insertGraph(lvDataAnalysis, pnlDataAnalysis, trackBar_hOffset.Value, trackBar_vOffset.Value, (double)numericUpDown_hScale.Value, (double)numericUpDown_vScale.Value, tbGraphLineColor.BackColor);
        }

        private void button_RemoveGraph_Click(object sender, EventArgs e)
        {
            DataAnalysisObj.removeGraph(lvDataAnalysis);
        }

        private void textBox_trackBarGraphVer_TextChanged(object sender, EventArgs e)
        {
            try
            {
                trackBar_vOffset.Value = Convert.ToInt16(textBox_trackBarGraphVer.Text);
                trackBar_vOffset_Scroll(this, null);
                textBox_trackBarGraphVer.BackColor = Color.White;
            }
            catch
            {
                textBox_trackBarGraphVer.BackColor = Color.Red;

            }
        }

        private void textBox_trackBarGraphHor_TextChanged(object sender, EventArgs e)
        {
            try
            {
                trackBar_hOffset.Value = Convert.ToInt16(textBox_trackBarGraphHor.Text);
                trackBar_hOffset_Scroll(this, null);
                textBox_trackBarGraphHor.BackColor = Color.White;
            }
            catch
            {
                textBox_trackBarGraphHor.BackColor = Color.Red;

            }
        }
   

        private void btnDataAnalysisSelectAll_Click(object sender, EventArgs e)
        {
            foreach (ListViewItem item in lvDataAnalysis.Items)
            {
                item.Checked = true;
            }
        }

        private void btnDataAnalysisDeselectAll_Click(object sender, EventArgs e)
        {

            foreach (ListViewItem item in lvDataAnalysis.Items)
            {
                item.Checked = false;
            }
        }

        private void lvDataTx_SelectedIndexChanged(object sender, EventArgs e)
        {
        }

        private void lvDataTx_ItemChecked(object sender, ItemCheckedEventArgs e)
        {
            foreach (ListViewItem item in lvDataTx.Items)
            {
                if(item.Checked)
                {
                    numericUpDownDataTx.Value = Convert.ToDecimal(item.SubItems[1].Text);
                    break;
                }
            }

        }        

        private void numericUpDownDataTx_ValueChanged(object sender, EventArgs e)
        {
            if (DataTxDisplayObj.setData(lvDataTx, ref MsgUdpT01.message, Convert.ToDouble(numericUpDownDataTx.Value)))
            {
                numericUpDownDataTx.BackColor = Color.White;
            }
            else
            {
                numericUpDownDataTx.BackColor = Color.Red;
            }
        }
    }
}

public static class ControlExtensions
{
    public static void DoubleBuffered(this Control control, bool enable)
    {
        var doubleBufferPropertyInfo = control.GetType().GetProperty("DoubleBuffered", BindingFlags.Instance | BindingFlags.NonPublic);
        doubleBufferPropertyInfo.SetValue(control, enable, null);
    }
}
