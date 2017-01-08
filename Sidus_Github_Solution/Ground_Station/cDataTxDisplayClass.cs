using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace Ground_Station
{
    class cDataTxDisplayClass
    {
        public void init(ListView lv, structMsgUdpT01 data)
        {
            lv.DoubleBuffered(true);

            foreach (var prop in data.GetType().GetProperties())
            {
                ListViewItem lvi = new ListViewItem(prop.Name);
                lvi.SubItems.Add(prop.GetValue(data, null).ToString());
                lv.Items.Add(lvi);
            }
         
        }
        public void update(ListView lv, structMsgUdpT01 data)
        {

            foreach (var prop in data.GetType().GetProperties())
            {
                ListViewItem lvi = lv.FindItemWithText(prop.Name);
                if (lvi != null)
                {
                    lvi.SubItems[1].Text = string.Format("{0,8:0.00}", Convert.ToDouble(prop.GetValue(data, null).ToString()));
                }
            }

        }

        public void setData(ListView lv, ref structMsgUdpT01 message, double data)
        {
            /*
            structMsgUdpT01 myMsg = new structMsgUdpT01();
            foreach (PropertyInfo prop in myMsg.GetType().GetProperties())
            {
                
                //    prop.SetValue(message, Convert.ChangeType(myByte, prop.PropertyType), null);

                PropertyInfo myProp = myMsg.GetType().GetProperty("pidRatePitchKd");
                myProp.SetValue(myMsg, Convert.ChangeType(5, prop.PropertyType), null);
              
                
            }
            
    */

            
        }
    }
}
