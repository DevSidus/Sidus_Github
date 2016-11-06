namespace Ground_Station
{
    partial class Ground_Station
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.bwUdpSniffer = new System.ComponentModel.BackgroundWorker();
            this.pnlTopPanel = new System.Windows.Forms.Panel();
            this.pnlHeartBeat = new System.Windows.Forms.Panel();
            this.pnlBottomPanel = new System.Windows.Forms.Panel();
            this.prbMainProgress = new System.Windows.Forms.ProgressBar();
            this.pnlContentPanel = new System.Windows.Forms.Panel();
            this.pnlTopPanel.SuspendLayout();
            this.pnlBottomPanel.SuspendLayout();
            this.SuspendLayout();
            // 
            // pnlTopPanel
            // 
            this.pnlTopPanel.Controls.Add(this.pnlHeartBeat);
            this.pnlTopPanel.Dock = System.Windows.Forms.DockStyle.Top;
            this.pnlTopPanel.Location = new System.Drawing.Point(0, 0);
            this.pnlTopPanel.Name = "pnlTopPanel";
            this.pnlTopPanel.Size = new System.Drawing.Size(1099, 10);
            this.pnlTopPanel.TabIndex = 0;
            // 
            // pnlHeartBeat
            // 
            this.pnlHeartBeat.BackColor = System.Drawing.Color.OrangeRed;
            this.pnlHeartBeat.Dock = System.Windows.Forms.DockStyle.Top;
            this.pnlHeartBeat.Location = new System.Drawing.Point(0, 0);
            this.pnlHeartBeat.Name = "pnlHeartBeat";
            this.pnlHeartBeat.Size = new System.Drawing.Size(1099, 10);
            this.pnlHeartBeat.TabIndex = 0;
            // 
            // pnlBottomPanel
            // 
            this.pnlBottomPanel.Controls.Add(this.prbMainProgress);
            this.pnlBottomPanel.Dock = System.Windows.Forms.DockStyle.Bottom;
            this.pnlBottomPanel.Location = new System.Drawing.Point(0, 471);
            this.pnlBottomPanel.Name = "pnlBottomPanel";
            this.pnlBottomPanel.Size = new System.Drawing.Size(1099, 32);
            this.pnlBottomPanel.TabIndex = 1;
            // 
            // prbMainProgress
            // 
            this.prbMainProgress.Dock = System.Windows.Forms.DockStyle.Fill;
            this.prbMainProgress.Location = new System.Drawing.Point(0, 0);
            this.prbMainProgress.Name = "prbMainProgress";
            this.prbMainProgress.Size = new System.Drawing.Size(1099, 32);
            this.prbMainProgress.TabIndex = 0;
            // 
            // pnlContentPanel
            // 
            this.pnlContentPanel.Dock = System.Windows.Forms.DockStyle.Fill;
            this.pnlContentPanel.Location = new System.Drawing.Point(0, 10);
            this.pnlContentPanel.Name = "pnlContentPanel";
            this.pnlContentPanel.Size = new System.Drawing.Size(1099, 461);
            this.pnlContentPanel.TabIndex = 2;
            // 
            // Ground_Station
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(1099, 503);
            this.Controls.Add(this.pnlContentPanel);
            this.Controls.Add(this.pnlBottomPanel);
            this.Controls.Add(this.pnlTopPanel);
            this.Name = "Ground_Station";
            this.Text = "Ground Station v0.1";
            this.Load += new System.EventHandler(this.Ground_Station_Load);
            this.pnlTopPanel.ResumeLayout(false);
            this.pnlBottomPanel.ResumeLayout(false);
            this.ResumeLayout(false);

        }

        #endregion

        private System.ComponentModel.BackgroundWorker bwUdpSniffer;
        private System.Windows.Forms.Panel pnlTopPanel;
        private System.Windows.Forms.Panel pnlBottomPanel;
        private System.Windows.Forms.Panel pnlContentPanel;
        private System.Windows.Forms.Panel pnlHeartBeat;
        private System.Windows.Forms.ProgressBar prbMainProgress;
    }
}

