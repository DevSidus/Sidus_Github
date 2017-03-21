using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Reflection;
using System.Runtime.InteropServices;
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

            bwUdpTransmit.WorkerReportsProgress = true;
            bwUdpTransmit.DoWork += new DoWorkEventHandler(bwUdpTransmit_DoWork);



        }
        private void bwUdpSniffer_ProgressChanged(object sender, ProgressChangedEventArgs e)
        {
            //prbMainProgress.Value = e.ProgressPercentage;
        }
        private void bwUdpSniffer_DoWork(object sender, DoWorkEventArgs e)
        {
            long counter = 0;
            while (true)
            {
                qgsUdp.ReceivePacket();
                if (qgsUdp.receivedDataFresh)
                {
                    byte[] buffer = qgsUdp.getReceivedData();
                    if (buffer.Length >= Marshal.SizeOf(MsgUdpR01.message))
                    {
                        Buffer.BlockCopy(buffer, 0, MsgUdpR01.dataBytes, 0, Marshal.SizeOf(MsgUdpR01.message));
                        counter = 0;
                    }
                }

                checkUdpClientStatus();
                
                if (counter < 100)
                {
                    pnlHeartBeat.BackColor = System.Drawing.Color.Green;
                }
                else
                {
                    pnlHeartBeat.BackColor = System.Drawing.Color.OrangeRed;
                }
                

                counter++;
                System.Threading.Thread.Sleep(10);
            }
        }

        private void bwUdpTransmit_DoWork(object sender, DoWorkEventArgs e)
        {
            while (true)
            {
                MsgUdpT01.getPacket();
                qgsUdp.SendPacket(MsgUdpT01.dataBytes, Marshal.SizeOf(MsgUdpT01.message));
                MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandNoAction);
                System.Threading.Thread.Sleep(100);
            }

        }

        private void Ground_Station_Load(object sender, EventArgs e)
        {
            DataAnalysisObj.init(lvDataAnalysis, pnlDataAnalysis);
            DataTxDisplayObj.init(lvDataTx, MsgUdpT01.message);

            bwUdpSniffer.RunWorkerAsync();
            bwUdpTransmit.RunWorkerAsync();

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
            MsgUdpR01.setPacket();

            //DataAnalysisObj.data.R1_baroAlt = MsgUdpR01.message.coWorkerTxPacket.baroAlt;
            //DataAnalysisObj.data.R1_baroTemp = MsgUdpR01.message.coWorkerTxPacket.baroTemp;
            //DataAnalysisObj.data.R1_batteryVoltageInBits = MsgUdpR01.message.coWorkerTxPacket.batteryVoltageInBits;
            //DataAnalysisObj.data.R1_compassHdg = MsgUdpR01.message.coWorkerTxPacket.compassHdg;
            //DataAnalysisObj.data.R1_mpuAccRealX = MsgUdpR01.message.coWorkerTxPacket.mpuAccRealX;
            //DataAnalysisObj.data.R1_mpuAccRealY = MsgUdpR01.message.coWorkerTxPacket.mpuAccRealY;
            //DataAnalysisObj.data.R1_mpuAccRealZ = MsgUdpR01.message.coWorkerTxPacket.mpuAccRealZ;
            //DataAnalysisObj.data.R1_mpuAccX = MsgUdpR01.message.coWorkerTxPacket.mpuAccX;
            //DataAnalysisObj.data.R1_mpuAccY = MsgUdpR01.message.coWorkerTxPacket.mpuAccY;
            //DataAnalysisObj.data.R1_mpuAccZ = MsgUdpR01.message.coWorkerTxPacket.mpuAccZ;
            DataAnalysisObj.data.R1_mpuGyroX = MsgUdpR01.message.coWorkerTxPacket.mpuGyroX;
            DataAnalysisObj.data.R1_mpuGyroY = MsgUdpR01.message.coWorkerTxPacket.mpuGyroY;
            DataAnalysisObj.data.R1_mpuGyroZ = MsgUdpR01.message.coWorkerTxPacket.mpuGyroZ;
            DataAnalysisObj.data.R1_mpuPitch = (float)(MsgUdpR01.message.coWorkerTxPacket.mpuPitch * 180.0 / Math.PI);
            DataAnalysisObj.data.R1_mpuRoll = (float)(MsgUdpR01.message.coWorkerTxPacket.mpuRoll * 180.0 / Math.PI);
            DataAnalysisObj.data.R1_mpuYaw = (float)(MsgUdpR01.message.coWorkerTxPacket.mpuYaw * 180.0 / Math.PI);

            DataAnalysisObj.data.R2_modeQuad = MsgUdpR01.message.serialR01RelayPacket.modeQuad;
            DataAnalysisObj.data.R2_rxThrottle = MsgUdpR01.message.serialR01RelayPacket.rxThrottle;
            DataAnalysisObj.data.R2_rxPitch = MsgUdpR01.message.serialR01RelayPacket.rxPitch;
            DataAnalysisObj.data.R2_rxRoll = MsgUdpR01.message.serialR01RelayPacket.rxRoll;
            DataAnalysisObj.data.R2_rxYaw = MsgUdpR01.message.serialR01RelayPacket.rxYaw;

            DataAnalysisObj.data.R2_pidRatePitchKp = MsgUdpR01.message.serialR01RelayPacket.pidRatePitchKp;
            DataAnalysisObj.data.R2_pidRatePitchKi = MsgUdpR01.message.serialR01RelayPacket.pidRatePitchKi;
            DataAnalysisObj.data.R2_pidRatePitchKd = MsgUdpR01.message.serialR01RelayPacket.pidRatePitchKd;
            DataAnalysisObj.data.R2_pidRatePitchOutput = MsgUdpR01.message.serialR01RelayPacket.pidRatePitchOutput;
            DataAnalysisObj.data.R2_pidRatePitchPresult = MsgUdpR01.message.serialR01RelayPacket.pidRatePitchPresult;
            DataAnalysisObj.data.R2_pidRatePitchIresult = MsgUdpR01.message.serialR01RelayPacket.pidRatePitchIresult;
            DataAnalysisObj.data.R2_pidRatePitchDresult = MsgUdpR01.message.serialR01RelayPacket.pidRatePitchDresult;
            DataAnalysisObj.data.R2_pidRatePitchF1 = MsgUdpR01.message.serialR01RelayPacket.pidRatePitchF1;
            DataAnalysisObj.data.R2_pidRatePitchF2 = MsgUdpR01.message.serialR01RelayPacket.pidRatePitchF2;

            DataAnalysisObj.data.R2_pidAnglePitchKp = MsgUdpR01.message.serialR01RelayPacket.pidAnglePitchKp;
            DataAnalysisObj.data.R2_pidAnglePitchKi = MsgUdpR01.message.serialR01RelayPacket.pidAnglePitchKi;
            DataAnalysisObj.data.R2_pidAnglePitchKd = MsgUdpR01.message.serialR01RelayPacket.pidAnglePitchKd;
            DataAnalysisObj.data.R2_pidAnglePitchOutput = MsgUdpR01.message.serialR01RelayPacket.pidAnglePitchOutput;
            DataAnalysisObj.data.R2_pidAnglePitchPresult = MsgUdpR01.message.serialR01RelayPacket.pidAnglePitchPresult;
            DataAnalysisObj.data.R2_pidAnglePitchIresult = MsgUdpR01.message.serialR01RelayPacket.pidAnglePitchIresult;
            DataAnalysisObj.data.R2_pidAnglePitchDresult = MsgUdpR01.message.serialR01RelayPacket.pidAnglePitchDresult;
            DataAnalysisObj.data.R2_pidAnglePitchF1 = MsgUdpR01.message.serialR01RelayPacket.pidAnglePitchF1;
            DataAnalysisObj.data.R2_pidAnglePitchF2 = MsgUdpR01.message.serialR01RelayPacket.pidAnglePitchF2;

            DataAnalysisObj.data.R2_pidRateRollKp = MsgUdpR01.message.serialR01RelayPacket.pidRateRollKp;
            DataAnalysisObj.data.R2_pidRateRollKi = MsgUdpR01.message.serialR01RelayPacket.pidRateRollKi;
            DataAnalysisObj.data.R2_pidRateRollKd = MsgUdpR01.message.serialR01RelayPacket.pidRateRollKd;
            DataAnalysisObj.data.R2_pidRateRollOutput = MsgUdpR01.message.serialR01RelayPacket.pidRateRollOutput;
            DataAnalysisObj.data.R2_pidRateRollPresult = MsgUdpR01.message.serialR01RelayPacket.pidRateRollPresult;
            DataAnalysisObj.data.R2_pidRateRollIresult = MsgUdpR01.message.serialR01RelayPacket.pidRateRollIresult;
            DataAnalysisObj.data.R2_pidRateRollDresult = MsgUdpR01.message.serialR01RelayPacket.pidRateRollDresult;
            DataAnalysisObj.data.R2_pidRateRollF1 = MsgUdpR01.message.serialR01RelayPacket.pidRateRollF1;
            DataAnalysisObj.data.R2_pidRateRollF2 = MsgUdpR01.message.serialR01RelayPacket.pidRateRollF2;

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
        
        private void tb_pid_angle_pitch_roll_kp_Scroll(object sender, EventArgs e)
        {
            
            toolTip1.SetToolTip(tb_pid_angle_pitch_roll_kp, tb_pid_angle_pitch_roll_kp.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyAnglePitchRoll);

            MsgUdpT01.message.pidAnglePitchRollKp = Convert.ToByte(tb_pid_angle_pitch_roll_kp.Value);
            MsgUdpT01.message.pidAnglePitchRollKi = Convert.ToByte(tb_pid_angle_pitch_roll_ki.Value);
            MsgUdpT01.message.pidAnglePitchRollKd = Convert.ToByte(tb_pid_angle_pitch_roll_kd.Value);
            MsgUdpT01.message.pidAnglePitchRollF1 = Convert.ToByte(tb_pid_angle_pitch_roll_f1.Value);
            MsgUdpT01.message.pidAnglePitchRollF2 = Convert.ToByte(tb_pid_angle_pitch_roll_f2.Value);
            MsgUdpT01.message.pidAnglePitchRollOutFilter = Convert.ToByte(tb_pid_angle_pitch_roll_out_filter.Value);
        }

        private void tb_pid_angle_pitch_roll_ki_Scroll(object sender, EventArgs e)
        {
            toolTip1.SetToolTip(tb_pid_angle_pitch_roll_ki, tb_pid_angle_pitch_roll_ki.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyAnglePitchRoll);
            MsgUdpT01.message.pidAnglePitchRollKp = Convert.ToByte(tb_pid_angle_pitch_roll_kp.Value);
            MsgUdpT01.message.pidAnglePitchRollKi = Convert.ToByte(tb_pid_angle_pitch_roll_ki.Value);
            MsgUdpT01.message.pidAnglePitchRollKd = Convert.ToByte(tb_pid_angle_pitch_roll_kd.Value);
            MsgUdpT01.message.pidAnglePitchRollF1 = Convert.ToByte(tb_pid_angle_pitch_roll_f1.Value);
            MsgUdpT01.message.pidAnglePitchRollF2 = Convert.ToByte(tb_pid_angle_pitch_roll_f2.Value);
            MsgUdpT01.message.pidAnglePitchRollOutFilter = Convert.ToByte(tb_pid_angle_pitch_roll_out_filter.Value);
        }

        private void tb_pid_angle_pitch_roll_kd_Scroll(object sender, EventArgs e)
        {
            toolTip1.SetToolTip(tb_pid_angle_pitch_roll_kd, tb_pid_angle_pitch_roll_kd.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyAnglePitchRoll);
            MsgUdpT01.message.pidAnglePitchRollKp = Convert.ToByte(tb_pid_angle_pitch_roll_kp.Value);
            MsgUdpT01.message.pidAnglePitchRollKi = Convert.ToByte(tb_pid_angle_pitch_roll_ki.Value);
            MsgUdpT01.message.pidAnglePitchRollKd = Convert.ToByte(tb_pid_angle_pitch_roll_kd.Value);
            MsgUdpT01.message.pidAnglePitchRollF1 = Convert.ToByte(tb_pid_angle_pitch_roll_f1.Value);
            MsgUdpT01.message.pidAnglePitchRollF2 = Convert.ToByte(tb_pid_angle_pitch_roll_f2.Value);
            MsgUdpT01.message.pidAnglePitchRollOutFilter = Convert.ToByte(tb_pid_angle_pitch_roll_out_filter.Value);
        }
        private void tb_pid_angle_pitch_roll_f1_Scroll(object sender, EventArgs e)
        {
            toolTip1.SetToolTip(tb_pid_angle_pitch_roll_f1, tb_pid_angle_pitch_roll_f1.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyAnglePitchRoll);
            MsgUdpT01.message.pidAnglePitchRollKp = Convert.ToByte(tb_pid_angle_pitch_roll_kp.Value);
            MsgUdpT01.message.pidAnglePitchRollKi = Convert.ToByte(tb_pid_angle_pitch_roll_ki.Value);
            MsgUdpT01.message.pidAnglePitchRollKd = Convert.ToByte(tb_pid_angle_pitch_roll_kd.Value);
            MsgUdpT01.message.pidAnglePitchRollF1 = Convert.ToByte(tb_pid_angle_pitch_roll_f1.Value);
            MsgUdpT01.message.pidAnglePitchRollF2 = Convert.ToByte(tb_pid_angle_pitch_roll_f2.Value);
            MsgUdpT01.message.pidAnglePitchRollOutFilter = Convert.ToByte(tb_pid_angle_pitch_roll_out_filter.Value);
        }
        private void tb_pid_angle_pitch_roll_f2_Scroll(object sender, EventArgs e)
        {
            toolTip1.SetToolTip(tb_pid_angle_pitch_roll_f2, tb_pid_angle_pitch_roll_f2.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyAnglePitchRoll);
            MsgUdpT01.message.pidAnglePitchRollKp = Convert.ToByte(tb_pid_angle_pitch_roll_kp.Value);
            MsgUdpT01.message.pidAnglePitchRollKi = Convert.ToByte(tb_pid_angle_pitch_roll_ki.Value);
            MsgUdpT01.message.pidAnglePitchRollKd = Convert.ToByte(tb_pid_angle_pitch_roll_kd.Value);
            MsgUdpT01.message.pidAnglePitchRollF1 = Convert.ToByte(tb_pid_angle_pitch_roll_f1.Value);
            MsgUdpT01.message.pidAnglePitchRollF2 = Convert.ToByte(tb_pid_angle_pitch_roll_f2.Value);
            MsgUdpT01.message.pidAnglePitchRollOutFilter = Convert.ToByte(tb_pid_angle_pitch_roll_out_filter.Value);
        }

        private void tb_pid_rate_pitch_roll_kp_Scroll(object sender, EventArgs e)
        {
            toolTip1.SetToolTip(tb_pid_rate_pitch_roll_kp, tb_pid_rate_pitch_roll_kp.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyRatePitchRoll);
            MsgUdpT01.message.pidRatePitchRollKp = Convert.ToByte(tb_pid_rate_pitch_roll_kp.Value);
            MsgUdpT01.message.pidRatePitchRollKi = Convert.ToByte(tb_pid_rate_pitch_roll_ki.Value);
            MsgUdpT01.message.pidRatePitchRollKd = Convert.ToByte(tb_pid_rate_pitch_roll_kd.Value);
            MsgUdpT01.message.pidRatePitchRollF1 = Convert.ToByte(tb_pid_rate_pitch_roll_f1.Value);
            MsgUdpT01.message.pidRatePitchRollF2 = Convert.ToByte(tb_pid_rate_pitch_roll_f2.Value);
        }

        private void tb_pid_rate_pitch_roll_ki_Scroll(object sender, EventArgs e)
        {
            toolTip1.SetToolTip(tb_pid_rate_pitch_roll_ki, tb_pid_rate_pitch_roll_ki.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyRatePitchRoll);
            MsgUdpT01.message.pidRatePitchRollKp = Convert.ToByte(tb_pid_rate_pitch_roll_kp.Value);
            MsgUdpT01.message.pidRatePitchRollKi = Convert.ToByte(tb_pid_rate_pitch_roll_ki.Value);
            MsgUdpT01.message.pidRatePitchRollKd = Convert.ToByte(tb_pid_rate_pitch_roll_kd.Value);
            MsgUdpT01.message.pidRatePitchRollF1 = Convert.ToByte(tb_pid_rate_pitch_roll_f1.Value);
            MsgUdpT01.message.pidRatePitchRollF2 = Convert.ToByte(tb_pid_rate_pitch_roll_f2.Value);
        }

        private void tb_pid_rate_pitch_roll_kd_Scroll(object sender, EventArgs e)
        {
            toolTip1.SetToolTip(tb_pid_rate_pitch_roll_kd, tb_pid_rate_pitch_roll_kd.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyRatePitchRoll);
            MsgUdpT01.message.pidRatePitchRollKp = Convert.ToByte(tb_pid_rate_pitch_roll_kp.Value);
            MsgUdpT01.message.pidRatePitchRollKi = Convert.ToByte(tb_pid_rate_pitch_roll_ki.Value);
            MsgUdpT01.message.pidRatePitchRollKd = Convert.ToByte(tb_pid_rate_pitch_roll_kd.Value);
            MsgUdpT01.message.pidRatePitchRollF1 = Convert.ToByte(tb_pid_rate_pitch_roll_f1.Value);
            MsgUdpT01.message.pidRatePitchRollF2 = Convert.ToByte(tb_pid_rate_pitch_roll_f2.Value);
        }
        
        private void tb_pid_rate_pitch_roll_f1_Scroll(object sender, EventArgs e)
        {
            toolTip1.SetToolTip(tb_pid_rate_pitch_roll_f1, tb_pid_rate_pitch_roll_f1.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyRatePitchRoll);
            MsgUdpT01.message.pidRatePitchRollKp = Convert.ToByte(tb_pid_rate_pitch_roll_kp.Value);
            MsgUdpT01.message.pidRatePitchRollKi = Convert.ToByte(tb_pid_rate_pitch_roll_ki.Value);
            MsgUdpT01.message.pidRatePitchRollKd = Convert.ToByte(tb_pid_rate_pitch_roll_kd.Value);
            MsgUdpT01.message.pidRatePitchRollF1 = Convert.ToByte(tb_pid_rate_pitch_roll_f1.Value);
            MsgUdpT01.message.pidRatePitchRollF2 = Convert.ToByte(tb_pid_rate_pitch_roll_f2.Value);
        }

        private void tb_pid_rate_pitch_roll_f2_Scroll(object sender, EventArgs e)
        {
            toolTip1.SetToolTip(tb_pid_rate_pitch_roll_f2, tb_pid_rate_pitch_roll_f2.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyRatePitchRoll);
            MsgUdpT01.message.pidRatePitchRollKp = Convert.ToByte(tb_pid_rate_pitch_roll_kp.Value);
            MsgUdpT01.message.pidRatePitchRollKi = Convert.ToByte(tb_pid_rate_pitch_roll_ki.Value);
            MsgUdpT01.message.pidRatePitchRollKd = Convert.ToByte(tb_pid_rate_pitch_roll_kd.Value);
            MsgUdpT01.message.pidRatePitchRollF1 = Convert.ToByte(tb_pid_rate_pitch_roll_f1.Value);
            MsgUdpT01.message.pidRatePitchRollF2 = Convert.ToByte(tb_pid_rate_pitch_roll_f2.Value);
        }

        private void tb_pid_angle_yaw_kp_Scroll(object sender, EventArgs e)
        {
            toolTip1.SetToolTip(tb_pid_angle_yaw_kp, tb_pid_angle_yaw_kp.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyAngleYaw);

            MsgUdpT01.message.pidAngleYawKp = Convert.ToByte(tb_pid_angle_yaw_kp.Value);
            MsgUdpT01.message.pidAngleYawKi = Convert.ToByte(tb_pid_angle_yaw_ki.Value);
            MsgUdpT01.message.pidAngleYawKd = Convert.ToByte(tb_pid_angle_yaw_kd.Value);
            MsgUdpT01.message.pidAngleYawF1 = Convert.ToByte(tb_pid_angle_yaw_f1.Value);
            MsgUdpT01.message.pidAngleYawF2 = Convert.ToByte(tb_pid_angle_yaw_f2.Value);
            MsgUdpT01.message.pidAngleYawOutFilter = Convert.ToByte(tb_pid_angle_yaw_out_filter.Value);
        }

        private void tb_pid_angle_yaw_ki_Scroll(object sender, EventArgs e)
        {
            toolTip1.SetToolTip(tb_pid_angle_yaw_ki, tb_pid_angle_yaw_ki.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyAngleYaw);

            MsgUdpT01.message.pidAngleYawKp = Convert.ToByte(tb_pid_angle_yaw_kp.Value);
            MsgUdpT01.message.pidAngleYawKi = Convert.ToByte(tb_pid_angle_yaw_ki.Value);
            MsgUdpT01.message.pidAngleYawKd = Convert.ToByte(tb_pid_angle_yaw_kd.Value);
            MsgUdpT01.message.pidAngleYawF1 = Convert.ToByte(tb_pid_angle_yaw_f1.Value);
            MsgUdpT01.message.pidAngleYawF2 = Convert.ToByte(tb_pid_angle_yaw_f2.Value);
            MsgUdpT01.message.pidAngleYawOutFilter = Convert.ToByte(tb_pid_angle_yaw_out_filter.Value);
        }

        private void tb_pid_angle_yaw_kd_Scroll(object sender, EventArgs e)
        {
            toolTip1.SetToolTip(tb_pid_angle_yaw_kd, tb_pid_angle_yaw_kd.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyAngleYaw);

            MsgUdpT01.message.pidAngleYawKp = Convert.ToByte(tb_pid_angle_yaw_kp.Value);
            MsgUdpT01.message.pidAngleYawKi = Convert.ToByte(tb_pid_angle_yaw_ki.Value);
            MsgUdpT01.message.pidAngleYawKd = Convert.ToByte(tb_pid_angle_yaw_kd.Value);
            MsgUdpT01.message.pidAngleYawF1 = Convert.ToByte(tb_pid_angle_yaw_f1.Value);
            MsgUdpT01.message.pidAngleYawF2 = Convert.ToByte(tb_pid_angle_yaw_f2.Value);
            MsgUdpT01.message.pidAngleYawOutFilter = Convert.ToByte(tb_pid_angle_yaw_out_filter.Value);
        }

        private void tb_pid_angle_yaw_f1_Scroll(object sender, EventArgs e)
        {
            toolTip1.SetToolTip(tb_pid_angle_yaw_f1, tb_pid_angle_yaw_f1.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyAngleYaw);

            MsgUdpT01.message.pidAngleYawKp = Convert.ToByte(tb_pid_angle_yaw_kp.Value);
            MsgUdpT01.message.pidAngleYawKi = Convert.ToByte(tb_pid_angle_yaw_ki.Value);
            MsgUdpT01.message.pidAngleYawKd = Convert.ToByte(tb_pid_angle_yaw_kd.Value);
            MsgUdpT01.message.pidAngleYawF1 = Convert.ToByte(tb_pid_angle_yaw_f1.Value);
            MsgUdpT01.message.pidAngleYawF2 = Convert.ToByte(tb_pid_angle_yaw_f2.Value);
            MsgUdpT01.message.pidAngleYawOutFilter = Convert.ToByte(tb_pid_angle_yaw_out_filter.Value);
        }

        private void tb_pid_angle_yaw_f2_Scroll(object sender, EventArgs e)
        {
            toolTip1.SetToolTip(tb_pid_angle_yaw_f2, tb_pid_angle_yaw_f2.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyAngleYaw);

            MsgUdpT01.message.pidAngleYawKp = Convert.ToByte(tb_pid_angle_yaw_kp.Value);
            MsgUdpT01.message.pidAngleYawKi = Convert.ToByte(tb_pid_angle_yaw_ki.Value);
            MsgUdpT01.message.pidAngleYawKd = Convert.ToByte(tb_pid_angle_yaw_kd.Value);
            MsgUdpT01.message.pidAngleYawF1 = Convert.ToByte(tb_pid_angle_yaw_f1.Value);
            MsgUdpT01.message.pidAngleYawF2 = Convert.ToByte(tb_pid_angle_yaw_f2.Value);
            MsgUdpT01.message.pidAngleYawOutFilter = Convert.ToByte(tb_pid_angle_yaw_out_filter.Value);
        }

        private void tb_pid_rate_yaw_kp_Scroll(object sender, EventArgs e)
        {
            toolTip1.SetToolTip(tb_pid_rate_yaw_kp, tb_pid_rate_yaw_kp.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyRateYaw);

            MsgUdpT01.message.pidRateYawKp = Convert.ToByte(tb_pid_rate_yaw_kp.Value);
            MsgUdpT01.message.pidRateYawKi = Convert.ToByte(tb_pid_rate_yaw_ki.Value);
            MsgUdpT01.message.pidRateYawKd = Convert.ToByte(tb_pid_rate_yaw_kd.Value);
            MsgUdpT01.message.pidRateYawF1 = Convert.ToByte(tb_pid_rate_yaw_f1.Value);
            MsgUdpT01.message.pidRateYawF2 = Convert.ToByte(tb_pid_rate_yaw_f2.Value);
        }

        private void tb_pid_rate_yaw_ki_Scroll(object sender, EventArgs e)
        {
            toolTip1.SetToolTip(tb_pid_rate_yaw_ki, tb_pid_rate_yaw_ki.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyRateYaw);

            MsgUdpT01.message.pidRateYawKp = Convert.ToByte(tb_pid_rate_yaw_kp.Value);
            MsgUdpT01.message.pidRateYawKi = Convert.ToByte(tb_pid_rate_yaw_ki.Value);
            MsgUdpT01.message.pidRateYawKd = Convert.ToByte(tb_pid_rate_yaw_kd.Value);
            MsgUdpT01.message.pidRateYawF1 = Convert.ToByte(tb_pid_rate_yaw_f1.Value);
            MsgUdpT01.message.pidRateYawF2 = Convert.ToByte(tb_pid_rate_yaw_f2.Value);
        }

        private void tb_pid_rate_yaw_kd_Scroll(object sender, EventArgs e)
        {
            toolTip1.SetToolTip(tb_pid_rate_yaw_kd, tb_pid_rate_yaw_kd.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyRateYaw);

            MsgUdpT01.message.pidRateYawKp = Convert.ToByte(tb_pid_rate_yaw_kp.Value);
            MsgUdpT01.message.pidRateYawKi = Convert.ToByte(tb_pid_rate_yaw_ki.Value);
            MsgUdpT01.message.pidRateYawKd = Convert.ToByte(tb_pid_rate_yaw_kd.Value);
            MsgUdpT01.message.pidRateYawF1 = Convert.ToByte(tb_pid_rate_yaw_f1.Value);
            MsgUdpT01.message.pidRateYawF2 = Convert.ToByte(tb_pid_rate_yaw_f2.Value);
        }

        private void tb_pid_rate_yaw_f1_Scroll(object sender, EventArgs e)
        {
            toolTip1.SetToolTip(tb_pid_rate_yaw_f1, tb_pid_rate_yaw_f1.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyRateYaw);

            MsgUdpT01.message.pidRateYawKp = Convert.ToByte(tb_pid_rate_yaw_kp.Value);
            MsgUdpT01.message.pidRateYawKi = Convert.ToByte(tb_pid_rate_yaw_ki.Value);
            MsgUdpT01.message.pidRateYawKd = Convert.ToByte(tb_pid_rate_yaw_kd.Value);
            MsgUdpT01.message.pidRateYawF1 = Convert.ToByte(tb_pid_rate_yaw_f1.Value);
            MsgUdpT01.message.pidRateYawF2 = Convert.ToByte(tb_pid_rate_yaw_f2.Value);
        }

        private void tb_pid_rate_yaw_f2_Scroll(object sender, EventArgs e)
        {
            toolTip1.SetToolTip(tb_pid_rate_yaw_f2, tb_pid_rate_yaw_f2.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyRateYaw);

            MsgUdpT01.message.pidRateYawKp = Convert.ToByte(tb_pid_rate_yaw_kp.Value);
            MsgUdpT01.message.pidRateYawKi = Convert.ToByte(tb_pid_rate_yaw_ki.Value);
            MsgUdpT01.message.pidRateYawKd = Convert.ToByte(tb_pid_rate_yaw_kd.Value);
            MsgUdpT01.message.pidRateYawF1 = Convert.ToByte(tb_pid_rate_yaw_f1.Value);
            MsgUdpT01.message.pidRateYawF2 = Convert.ToByte(tb_pid_rate_yaw_f2.Value);
        }

        private void tb_pid_angle_pitch_roll_out_filter_Scroll(object sender, EventArgs e)
        {
            toolTip1.SetToolTip(tb_pid_angle_pitch_roll_out_filter, tb_pid_angle_pitch_roll_out_filter.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyAnglePitchRoll);
            MsgUdpT01.message.pidAnglePitchRollKp = Convert.ToByte(tb_pid_angle_pitch_roll_kp.Value);
            MsgUdpT01.message.pidAnglePitchRollKi = Convert.ToByte(tb_pid_angle_pitch_roll_ki.Value);
            MsgUdpT01.message.pidAnglePitchRollKd = Convert.ToByte(tb_pid_angle_pitch_roll_kd.Value);
            MsgUdpT01.message.pidAnglePitchRollF1 = Convert.ToByte(tb_pid_angle_pitch_roll_f1.Value);
            MsgUdpT01.message.pidAnglePitchRollF2 = Convert.ToByte(tb_pid_angle_pitch_roll_f2.Value);
            MsgUdpT01.message.pidAnglePitchRollOutFilter = Convert.ToByte(tb_pid_angle_pitch_roll_out_filter.Value);
        }

        private void tb_pid_angle_yaw_out_filter_Scroll(object sender, EventArgs e)
        {
            toolTip1.SetToolTip(tb_pid_angle_yaw_out_filter, tb_pid_angle_yaw_out_filter.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyAngleYaw);

            MsgUdpT01.message.pidAngleYawKp = Convert.ToByte(tb_pid_angle_yaw_kp.Value);
            MsgUdpT01.message.pidAngleYawKi = Convert.ToByte(tb_pid_angle_yaw_ki.Value);
            MsgUdpT01.message.pidAngleYawKd = Convert.ToByte(tb_pid_angle_yaw_kd.Value);
            MsgUdpT01.message.pidAngleYawF1 = Convert.ToByte(tb_pid_angle_yaw_f1.Value);
            MsgUdpT01.message.pidAngleYawF2 = Convert.ToByte(tb_pid_angle_yaw_f2.Value);
            MsgUdpT01.message.pidAngleYawOutFilter = Convert.ToByte(tb_pid_angle_yaw_out_filter.Value);
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
