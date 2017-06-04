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
        long udp_rx_reset_counter = 0;
        String textFileName = "";
        DateTime startTime=DateTime.Now;
        TimeSpan deltaTime;


        double myVal = 0;


        cDataAnalysisClass DataAnalysisObj = new cDataAnalysisClass();
        cDataTxDisplayClass DataTxDisplayObj = new cDataTxDisplayClass();

        cKalmanFilter attitudeKalmanObj = new cKalmanFilter(17.5f, 0, 0, 4.0f, 1.0f, 0.035f);
        public Ground_Station()
        {
            vfnDeclaration();
        }        
        public void vfnDeclaration()
        {
            //Initialize Form
            InitializeComponent();

            bwUdpTransmit.WorkerReportsProgress = true;
            bwUdpTransmit.DoWork += new DoWorkEventHandler(bwUdpTransmit_DoWork);


            bwUdpReceive.WorkerReportsProgress = true;
            bwUdpReceive.DoWork += new DoWorkEventHandler(bwUdpReceive_DoWork);

        }

        private void bwUdpTransmit_DoWork(object sender, DoWorkEventArgs e)
        {
            while (true)
            {
                MsgUdpT01.getPacket();
                qgsUdp.SendPacket(MsgUdpT01.dataBytes, Marshal.SizeOf(MsgUdpT01.message));
                MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandNoAction);
                System.Threading.Thread.Sleep(200);
            }

        }

        private void Ground_Station_Load(object sender, EventArgs e)
        {
            DataAnalysisObj.init(lvDataAnalysis, pnlDataAnalysis);
            DataTxDisplayObj.init(lvDataTx, MsgUdpT01.message);
            
            bwUdpTransmit.RunWorkerAsync();
            bwUdpReceive.RunWorkerAsync();

            timerDisplayRefresh.Enabled = true;

            ssMainLabel1.Text = "IP:" + qgsUdp.GetLocalIPv4();
            ssMainLabel2.Text = "Port#:" + qgsUdp.port.ToString();

            textFileName = DateTime.Now.Year.ToString() + "-" + DateTime.Now.Month.ToString() + "-" + DateTime.Now.Day.ToString() + "_" + DateTime.Now.Hour.ToString() + "-" + DateTime.Now.Minute.ToString() + "-" + DateTime.Now.Second.ToString() + ".txt";


            String insertLine = " ";
            foreach (var prop in DataAnalysisObj.data.GetType().GetProperties())
            {
                insertLine = insertLine + prop.Name + "  ";

            }
            using (System.IO.StreamWriter file = new System.IO.StreamWriter(@textFileName, true))
            {
                file.WriteLine(insertLine);
            }

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
            checkUdpClientStatus();
            DataAnalysisObj.update(lvDataAnalysis, pnlDataAnalysis, tbGraphBackColor.BackColor);
            DataTxDisplayObj.update(lvDataTx, MsgUdpT01.message);

            textBox1.Text = myVal.ToString("0000");
        }

        private void writeToTextFile()
        {
            String insertLine = " ";
            foreach (var prop in DataAnalysisObj.data.GetType().GetProperties())
            {

                insertLine = insertLine + string.Format("{0,8:0.00}", Convert.ToDouble(prop.GetValue(DataAnalysisObj.data, null).ToString())) + "  ";

            }


            using (System.IO.StreamWriter file = new System.IO.StreamWriter(@textFileName, true))
            {
                file.WriteLine(insertLine);
            }

        }

        private void updateMessagesForDataAnalysis()
        {
            MsgUdpR01.setPacket();

            //DataAnalysisObj.data.baroTemp = MsgUdpR01.message.baroTemp;
            //DataAnalysisObj.data.compassHdg = MsgUdpR01.message.compassHdg;

            DataAnalysisObj.data.timeStamp = MsgUdpR01.message.timeStamp;
            DataAnalysisObj.data.statusGS = MsgUdpR01.message.statusGS;
            DataAnalysisObj.data.mpuGyroX = MsgUdpR01.message.mpuGyroX;
            DataAnalysisObj.data.mpuGyroY = MsgUdpR01.message.mpuGyroY;
            DataAnalysisObj.data.mpuGyroZ = MsgUdpR01.message.mpuGyroZ;

            DataAnalysisObj.data.mpuAccX = MsgUdpR01.message.mpuAccX;
            DataAnalysisObj.data.mpuAccY = MsgUdpR01.message.mpuAccY;
            DataAnalysisObj.data.mpuAccZ = MsgUdpR01.message.mpuAccZ;
            
            DataAnalysisObj.data.mpuPitch = (float)(MsgUdpR01.message.mpuPitch * 180.0 / Math.PI);
            DataAnalysisObj.data.mpuRoll = (float)(MsgUdpR01.message.mpuRoll * 180.0 / Math.PI);
            DataAnalysisObj.data.mpuYaw = (float)(MsgUdpR01.message.mpuYaw * 180.0 / Math.PI);
            DataAnalysisObj.data.baroAlt = MsgUdpR01.message.baroAlt-800;
            DataAnalysisObj.data.batteryVoltageInBits = MsgUdpR01.message.batteryVoltageInBits; //(BAT_VOLT_DIV_R1 + BAT_VOLT_DIV_R2) / BAT_VOLT_DIV_R2; ;
            DataAnalysisObj.data.mpuAccWorldX = MsgUdpR01.message.mpuAccWorldX;
            DataAnalysisObj.data.mpuAccWorldY = MsgUdpR01.message.mpuAccWorldY;
            DataAnalysisObj.data.mpuAccWorldZ = MsgUdpR01.message.mpuAccWorldZ;
            DataAnalysisObj.data.quadVelocityWorldZ = (float)MsgUdpR01.message.quadVelocityWorldZ;
            DataAnalysisObj.data.quadPositionWorldZ = (float)MsgUdpR01.message.quadPositionWorldZ-800;
            DataAnalysisObj.data.modeQuad = MsgUdpR01.message.modeQuad;
            DataAnalysisObj.data.autoModeStatus = MsgUdpR01.message.autoModeStatus;
            DataAnalysisObj.data.rxThrottle = MsgUdpR01.message.rxThrottle;
            DataAnalysisObj.data.rxPitch = MsgUdpR01.message.rxPitch;
            DataAnalysisObj.data.rxRoll = MsgUdpR01.message.rxRoll;
            DataAnalysisObj.data.rxYaw = MsgUdpR01.message.rxYaw;
            DataAnalysisObj.data.pidRatePitchKp = MsgUdpR01.message.pidRatePitchKp;
            DataAnalysisObj.data.pidRatePitchKi = MsgUdpR01.message.pidRatePitchKi;
            DataAnalysisObj.data.pidRatePitchKd = MsgUdpR01.message.pidRatePitchKd;
            DataAnalysisObj.data.pidRatePitchOutput = MsgUdpR01.message.pidRatePitchOutput;
            DataAnalysisObj.data.pidRatePitchPresult = MsgUdpR01.message.pidRatePitchPresult;
            DataAnalysisObj.data.pidRatePitchIresult = MsgUdpR01.message.pidRatePitchIresult;
            DataAnalysisObj.data.pidRatePitchDresult = MsgUdpR01.message.pidRatePitchDresult;
            DataAnalysisObj.data.pidRatePitchF1 =  MsgUdpR01.message.pidRatePitchF1;
            DataAnalysisObj.data.pidRatePitchF2 =  MsgUdpR01.message.pidRatePitchF2;
            DataAnalysisObj.data.pidAnglePitchKp = MsgUdpR01.message.pidAnglePitchKp;
            DataAnalysisObj.data.pidAnglePitchKi = MsgUdpR01.message.pidAnglePitchKi;
            DataAnalysisObj.data.pidAnglePitchKd = MsgUdpR01.message.pidAnglePitchKd;
            DataAnalysisObj.data.pidAnglePitchOutput = MsgUdpR01.message.pidAnglePitchOutput;
            DataAnalysisObj.data.pidAnglePitchPresult = MsgUdpR01.message.pidAnglePitchPresult;
            DataAnalysisObj.data.pidAnglePitchIresult = MsgUdpR01.message.pidAnglePitchIresult;
            DataAnalysisObj.data.pidAnglePitchDresult = MsgUdpR01.message.pidAnglePitchDresult;
            DataAnalysisObj.data.pidAnglePitchF1 = MsgUdpR01.message.pidAnglePitchF1;
            DataAnalysisObj.data.pidAnglePitchF2 = MsgUdpR01.message.pidAnglePitchF2;
            DataAnalysisObj.data.pidAnglePitchOutFilter = MsgUdpR01.message.pidAnglePitchOutFilter;
            DataAnalysisObj.data.pidRateRollKp =   MsgUdpR01.message.pidRateRollKp;
            DataAnalysisObj.data.pidRateRollKi =   MsgUdpR01.message.pidRateRollKi;
            DataAnalysisObj.data.pidRateRollKd =   MsgUdpR01.message.pidRateRollKd;
            DataAnalysisObj.data.pidRateRollOutput = MsgUdpR01.message.pidRateRollOutput;
            DataAnalysisObj.data.pidRateRollPresult = MsgUdpR01.message.pidRateRollPresult;
            DataAnalysisObj.data.pidRateRollIresult = MsgUdpR01.message.pidRateRollIresult;
            DataAnalysisObj.data.pidRateRollDresult = MsgUdpR01.message.pidRateRollDresult;
            DataAnalysisObj.data.pidRateRollF1 =   MsgUdpR01.message.pidRateRollF1;
            DataAnalysisObj.data.pidRateRollF2 =   MsgUdpR01.message.pidRateRollF2;
            DataAnalysisObj.data.pidAngleRollKp =  MsgUdpR01.message.pidAngleRollKp;
            DataAnalysisObj.data.pidAngleRollKi =  MsgUdpR01.message.pidAngleRollKi;
            DataAnalysisObj.data.pidAngleRollKd =  MsgUdpR01.message.pidAngleRollKd;
            DataAnalysisObj.data.pidAngleRollOutput = MsgUdpR01.message.pidAngleRollOutput;
            DataAnalysisObj.data.pidAngleRollPresult = MsgUdpR01.message.pidAngleRollPresult;
            DataAnalysisObj.data.pidAngleRollIresult = MsgUdpR01.message.pidAngleRollIresult;
            DataAnalysisObj.data.pidAngleRollDresult = MsgUdpR01.message.pidAngleRollDresult;
            DataAnalysisObj.data.pidAngleRollF1 =  MsgUdpR01.message.pidAngleRollF1;
            DataAnalysisObj.data.pidAngleRollF2 =  MsgUdpR01.message.pidAngleRollF2;
            DataAnalysisObj.data.pidAngleRollOutFilter = MsgUdpR01.message.pidAngleRollOutFilter;
            DataAnalysisObj.data.pidRateYawKp =    MsgUdpR01.message.pidRateYawKp;
            DataAnalysisObj.data.pidRateYawKi =    MsgUdpR01.message.pidRateYawKi;
            DataAnalysisObj.data.pidRateYawKd =    MsgUdpR01.message.pidRateYawKd;
            DataAnalysisObj.data.pidRateYawOutput = MsgUdpR01.message.pidRateYawOutput;
            DataAnalysisObj.data.pidRateYawPresult = MsgUdpR01.message.pidRateYawPresult;
            DataAnalysisObj.data.pidRateYawIresult = MsgUdpR01.message.pidRateYawIresult;
            DataAnalysisObj.data.pidRateYawDresult = MsgUdpR01.message.pidRateYawDresult;
            DataAnalysisObj.data.pidRateYawF1 =    MsgUdpR01.message.pidRateYawF1;
            DataAnalysisObj.data.pidRateYawF2 =    MsgUdpR01.message.pidRateYawF2;
            DataAnalysisObj.data.pidAngleYawKp =   MsgUdpR01.message.pidAngleYawKp;
            DataAnalysisObj.data.pidAngleYawKi =   MsgUdpR01.message.pidAngleYawKi;
            DataAnalysisObj.data.pidAngleYawKd =   MsgUdpR01.message.pidAngleYawKd;
            DataAnalysisObj.data.pidAngleYawOutput = MsgUdpR01.message.pidAngleYawOutput;
            DataAnalysisObj.data.pidAngleYawPresult = MsgUdpR01.message.pidAngleYawPresult;
            DataAnalysisObj.data.pidAngleYawIresult = MsgUdpR01.message.pidAngleYawIresult;
            DataAnalysisObj.data.pidAngleYawDresult = MsgUdpR01.message.pidAngleYawDresult;
            DataAnalysisObj.data.pidAngleYawF1 =   MsgUdpR01.message.pidAngleYawF1;
            DataAnalysisObj.data.pidAngleYawF2 =   MsgUdpR01.message.pidAngleYawF2;
            DataAnalysisObj.data.pidAngleYawOutFilter = MsgUdpR01.message.pidAngleYawOutFilter;
            DataAnalysisObj.data.commandedYawAngle = MsgUdpR01.message.commandedYawAngle;

            DataAnalysisObj.data.pidVelAltKp = MsgUdpR01.message.pidVelAltKp;
            DataAnalysisObj.data.pidVelAltKi = MsgUdpR01.message.pidVelAltKi;
            DataAnalysisObj.data.pidVelAltKd = MsgUdpR01.message.pidVelAltKd;
            DataAnalysisObj.data.pidVelAltOutput = MsgUdpR01.message.pidVelAltOutput;
            DataAnalysisObj.data.pidVelAltPresult = MsgUdpR01.message.pidVelAltPresult;
            DataAnalysisObj.data.pidVelAltIresult = MsgUdpR01.message.pidVelAltIresult;
            DataAnalysisObj.data.pidVelAltDresult = MsgUdpR01.message.pidVelAltDresult;
            DataAnalysisObj.data.pidVelAltF1 = MsgUdpR01.message.pidVelAltF1;
            DataAnalysisObj.data.pidVelAltF2 = MsgUdpR01.message.pidVelAltF2;
            DataAnalysisObj.data.pidAccAltKp = MsgUdpR01.message.pidAccAltKp;
            DataAnalysisObj.data.pidAccAltKi = MsgUdpR01.message.pidAccAltKi;
            DataAnalysisObj.data.pidAccAltKd = MsgUdpR01.message.pidAccAltKd;
            DataAnalysisObj.data.pidAccAltOutput = MsgUdpR01.message.pidAccAltOutput;
            DataAnalysisObj.data.pidAccAltPresult = MsgUdpR01.message.pidAccAltPresult;
            DataAnalysisObj.data.pidAccAltIresult = MsgUdpR01.message.pidAccAltIresult;
            DataAnalysisObj.data.pidAccAltDresult = MsgUdpR01.message.pidAccAltDresult;
            DataAnalysisObj.data.pidAccAltF1 = MsgUdpR01.message.pidAccAltF1;
            DataAnalysisObj.data.pidAccAltF2 = MsgUdpR01.message.pidAccAltF2;
            DataAnalysisObj.data.pidAccAltOutFilter = MsgUdpR01.message.pidAccAltOutFilter;


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

        private void tb_pid_acc_alt_kp_Scroll(object sender, EventArgs e)
        {
            toolTip1.SetToolTip(tb_pid_acc_alt_kp, tb_pid_acc_alt_kp.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyAccAlt);

            MsgUdpT01.message.pidAccAltKp = Convert.ToByte(tb_pid_acc_alt_kp.Value);
            MsgUdpT01.message.pidAccAltKi = Convert.ToByte(tb_pid_acc_alt_ki.Value);
            MsgUdpT01.message.pidAccAltKd = Convert.ToByte(tb_pid_acc_alt_kd.Value);
            MsgUdpT01.message.pidAccAltF1 = Convert.ToByte(tb_pid_acc_alt_f1.Value);
            MsgUdpT01.message.pidAccAltF2 = Convert.ToByte(tb_pid_acc_alt_f2.Value);
        }

        private void tb_pid_acc_alt_ki_Scroll(object sender, EventArgs e)
        {
            toolTip1.SetToolTip(tb_pid_acc_alt_ki, tb_pid_acc_alt_ki.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyAccAlt);

            MsgUdpT01.message.pidAccAltKp = Convert.ToByte(tb_pid_acc_alt_kp.Value);
            MsgUdpT01.message.pidAccAltKi = Convert.ToByte(tb_pid_acc_alt_ki.Value);
            MsgUdpT01.message.pidAccAltKd = Convert.ToByte(tb_pid_acc_alt_kd.Value);
            MsgUdpT01.message.pidAccAltF1 = Convert.ToByte(tb_pid_acc_alt_f1.Value);
            MsgUdpT01.message.pidAccAltF2 = Convert.ToByte(tb_pid_acc_alt_f2.Value);
        }

        private void tb_pid_acc_alt_kd_Scroll(object sender, EventArgs e)
        {

            toolTip1.SetToolTip(tb_pid_acc_alt_kd, tb_pid_acc_alt_kd.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyAccAlt);

            MsgUdpT01.message.pidAccAltKp = Convert.ToByte(tb_pid_acc_alt_kp.Value);
            MsgUdpT01.message.pidAccAltKi = Convert.ToByte(tb_pid_acc_alt_ki.Value);
            MsgUdpT01.message.pidAccAltKd = Convert.ToByte(tb_pid_acc_alt_kd.Value);
            MsgUdpT01.message.pidAccAltF1 = Convert.ToByte(tb_pid_acc_alt_f1.Value);
            MsgUdpT01.message.pidAccAltF2 = Convert.ToByte(tb_pid_acc_alt_f2.Value);
        }

        private void tb_pid_acc_alt_f1_Scroll(object sender, EventArgs e)
        {

            toolTip1.SetToolTip(tb_pid_acc_alt_f1, tb_pid_acc_alt_f1.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyAccAlt);

            MsgUdpT01.message.pidAccAltKp = Convert.ToByte(tb_pid_acc_alt_kp.Value);
            MsgUdpT01.message.pidAccAltKi = Convert.ToByte(tb_pid_acc_alt_ki.Value);
            MsgUdpT01.message.pidAccAltKd = Convert.ToByte(tb_pid_acc_alt_kd.Value);
            MsgUdpT01.message.pidAccAltF1 = Convert.ToByte(tb_pid_acc_alt_f1.Value);
            MsgUdpT01.message.pidAccAltF2 = Convert.ToByte(tb_pid_acc_alt_f2.Value);
        }

        private void tb_pid_acc_alt_f2_Scroll(object sender, EventArgs e)
        {

            toolTip1.SetToolTip(tb_pid_acc_alt_f2, tb_pid_acc_alt_f2.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyAccAlt);

            MsgUdpT01.message.pidAccAltKp = Convert.ToByte(tb_pid_acc_alt_kp.Value);
            MsgUdpT01.message.pidAccAltKi = Convert.ToByte(tb_pid_acc_alt_ki.Value);
            MsgUdpT01.message.pidAccAltKd = Convert.ToByte(tb_pid_acc_alt_kd.Value);
            MsgUdpT01.message.pidAccAltF1 = Convert.ToByte(tb_pid_acc_alt_f1.Value);
            MsgUdpT01.message.pidAccAltF2 = Convert.ToByte(tb_pid_acc_alt_f2.Value);
        }

        private void tb_pid_vel_alt_kp_Scroll(object sender, EventArgs e)
        {
            toolTip1.SetToolTip(tb_pid_vel_alt_kp, tb_pid_vel_alt_kp.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyVelAlt);

            MsgUdpT01.message.pidVelAltKp = Convert.ToByte(tb_pid_vel_alt_kp.Value);
            MsgUdpT01.message.pidVelAltKi = Convert.ToByte(tb_pid_vel_alt_ki.Value);
            MsgUdpT01.message.pidVelAltKd = Convert.ToByte(tb_pid_vel_alt_kd.Value);
            MsgUdpT01.message.pidVelAltF1 = Convert.ToByte(tb_pid_vel_alt_f1.Value);
            MsgUdpT01.message.pidVelAltF2 = Convert.ToByte(tb_pid_vel_alt_f2.Value);
            MsgUdpT01.message.pidVelAltOutFilter = Convert.ToByte(tb_pid_vel_alt_out_filter.Value);
        }

        private void tb_pid_vel_alt_ki_Scroll(object sender, EventArgs e)
        {
            toolTip1.SetToolTip(tb_pid_vel_alt_ki, tb_pid_vel_alt_ki.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyVelAlt);

            MsgUdpT01.message.pidVelAltKp = Convert.ToByte(tb_pid_vel_alt_kp.Value);
            MsgUdpT01.message.pidVelAltKi = Convert.ToByte(tb_pid_vel_alt_ki.Value);
            MsgUdpT01.message.pidVelAltKd = Convert.ToByte(tb_pid_vel_alt_kd.Value);
            MsgUdpT01.message.pidVelAltF1 = Convert.ToByte(tb_pid_vel_alt_f1.Value);
            MsgUdpT01.message.pidVelAltF2 = Convert.ToByte(tb_pid_vel_alt_f2.Value);
            MsgUdpT01.message.pidVelAltOutFilter = Convert.ToByte(tb_pid_vel_alt_out_filter.Value);
        }

        private void tb_pid_vel_alt_kd_Scroll(object sender, EventArgs e)
        {

            toolTip1.SetToolTip(tb_pid_vel_alt_kd, tb_pid_vel_alt_kd.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyVelAlt);

            MsgUdpT01.message.pidVelAltKp = Convert.ToByte(tb_pid_vel_alt_kp.Value);
            MsgUdpT01.message.pidVelAltKi = Convert.ToByte(tb_pid_vel_alt_ki.Value);
            MsgUdpT01.message.pidVelAltKd = Convert.ToByte(tb_pid_vel_alt_kd.Value);
            MsgUdpT01.message.pidVelAltF1 = Convert.ToByte(tb_pid_vel_alt_f1.Value);
            MsgUdpT01.message.pidVelAltF2 = Convert.ToByte(tb_pid_vel_alt_f2.Value);
            MsgUdpT01.message.pidVelAltOutFilter = Convert.ToByte(tb_pid_vel_alt_out_filter.Value);
        }

        private void tb_pid_vel_alt_f1_Scroll(object sender, EventArgs e)
        {

            toolTip1.SetToolTip(tb_pid_vel_alt_f1, tb_pid_vel_alt_f1.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyVelAlt);

            MsgUdpT01.message.pidVelAltKp = Convert.ToByte(tb_pid_vel_alt_kp.Value);
            MsgUdpT01.message.pidVelAltKi = Convert.ToByte(tb_pid_vel_alt_ki.Value);
            MsgUdpT01.message.pidVelAltKd = Convert.ToByte(tb_pid_vel_alt_kd.Value);
            MsgUdpT01.message.pidVelAltF1 = Convert.ToByte(tb_pid_vel_alt_f1.Value);
            MsgUdpT01.message.pidVelAltF2 = Convert.ToByte(tb_pid_vel_alt_f2.Value);
            MsgUdpT01.message.pidVelAltOutFilter = Convert.ToByte(tb_pid_vel_alt_out_filter.Value);
        }

        private void tb_pid_vel_alt_f2_Scroll(object sender, EventArgs e)
        {

            toolTip1.SetToolTip(tb_pid_vel_alt_f2, tb_pid_vel_alt_f2.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyVelAlt);

            MsgUdpT01.message.pidVelAltKp = Convert.ToByte(tb_pid_vel_alt_kp.Value);
            MsgUdpT01.message.pidVelAltKi = Convert.ToByte(tb_pid_vel_alt_ki.Value);
            MsgUdpT01.message.pidVelAltKd = Convert.ToByte(tb_pid_vel_alt_kd.Value);
            MsgUdpT01.message.pidVelAltF1 = Convert.ToByte(tb_pid_vel_alt_f1.Value);
            MsgUdpT01.message.pidVelAltF2 = Convert.ToByte(tb_pid_vel_alt_f2.Value);
            MsgUdpT01.message.pidVelAltOutFilter = Convert.ToByte(tb_pid_vel_alt_out_filter.Value);
        }

        private void cb_AltitudeHold_CheckedChanged(object sender, EventArgs e)
        {
            if(cb_AltitudeHold.Checked)
            {
                MsgUdpT01.message.autoModeCommand = Convert.ToByte(autoModeType.autoModeAltitude);
            }
            else
            {
                MsgUdpT01.message.autoModeCommand = Convert.ToByte(autoModeType.autoModeOFF);
            }
        }

        private void tb_pid_vel_alt_out_filter_Scroll(object sender, EventArgs e)
        {

            toolTip1.SetToolTip(tb_pid_vel_alt_out_filter, tb_pid_vel_alt_out_filter.Value.ToString());
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyVelAlt);

            MsgUdpT01.message.pidVelAltKp = Convert.ToByte(tb_pid_vel_alt_kp.Value);
            MsgUdpT01.message.pidVelAltKi = Convert.ToByte(tb_pid_vel_alt_ki.Value);
            MsgUdpT01.message.pidVelAltKd = Convert.ToByte(tb_pid_vel_alt_kd.Value);
            MsgUdpT01.message.pidVelAltF1 = Convert.ToByte(tb_pid_vel_alt_f1.Value);
            MsgUdpT01.message.pidVelAltF2 = Convert.ToByte(tb_pid_vel_alt_f2.Value);
            MsgUdpT01.message.pidVelAltOutFilter = Convert.ToByte(tb_pid_vel_alt_out_filter.Value);
        }

        private void button_setallcmd_Click(object sender, EventArgs e)
        {

            tb_pid_angle_pitch_roll_kp_Scroll(this, e);
            tb_pid_rate_pitch_roll_kp_Scroll(this, e);
            tb_pid_angle_yaw_kp_Scroll(this, e);
            tb_pid_rate_yaw_kp_Scroll(this, e);
            tb_pid_vel_alt_kp_Scroll(this, e);
            tb_pid_acc_alt_kp_Scroll(this, e);
            MsgUdpT01.message.pidCommandState = Convert.ToByte(pidCommandType.pidCommandApplyAll);
        }
        

        private void bwUdpReceive_DoWork(object sender, DoWorkEventArgs e)
        {
            while (true)
            {
                qgsUdp.ReceivePacket();
                if (qgsUdp.receivedDataFresh)
                {
                    byte[] buffer = qgsUdp.getReceivedData();
                    try
                    {
                        if (buffer.Length >= Marshal.SizeOf(MsgUdpR01.message))
                        {
                            Buffer.BlockCopy(buffer, 0, MsgUdpR01.dataBytes, 0, Marshal.SizeOf(MsgUdpR01.message));
                            udp_rx_reset_counter = 0;
                            updateMessagesForDataAnalysis();
                            writeToTextFile();
                        }
                    }
                    catch
                    {
                    }


                }
                udp_rx_reset_counter++;

                deltaTime = DateTime.Now - startTime;
                myVal = deltaTime.TotalMilliseconds;
                startTime = DateTime.Now;
                //System.Threading.Thread.Sleep(1);
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
