#include "pid.h"
#include "rp_math.h"
/**
 *  @name   single_pid_ctrl
 */
void single_pid_ctrl(pid_ctrl_t *pid)
{
    // 濞ｅ洦绻傞悺銊ф嫚椤栨碍鈻曢柛濠忔嫹(闂傚洠鍋撻悷鏇氱濠€顏呭緞閺嶎厽妗ㄩ柤濂変海椤㈡垹鎷嬮敍鍕毈閻犲浂鍨板Ο锟�)
	//pid->err = pid->target-pid->measure;
	  pid->integral += pid->err;  
    pid->integral = constrain(pid->integral, -pid->integral_max, +pid->integral_max);
    // p i d 閺夊牊鎸搁崵顓熴亜绾拋鍚€缂佺媴鎷�
    pid->pout = pid->kp * pid->err;
    pid->iout = pid->ki * pid->integral;
	  pid->dout = pid->kd * (pid->err - pid->last_err);
	  pid->last_dout=pid->dout;
    // 缂侀硸鍨版慨鐎檌d閺夊牊鎸搁崵顓㈠磹閿燂拷
    pid->out = pid->pout + pid->iout + pid->dout;
    pid->out = constrain(pid->out, -pid->out_max, pid->out_max);
    // 閻犱焦婢樼紞宥嗙▔婵犲喚鍋ч悹鍥跺灠濡﹪宕愰敓锟�
    pid->last_err = pid->err;
}


/**
 *	@brief	pid闁诡剛绮敮鍫曞礆閿燂拷 闁告瑥鍊归弳鐔兼晬濮橆剦妯嗛柣婊愭嫹 闁告劕鎳愰獮锟�  濠㈣埖鐗滈獮鍡涘箣閺嵮冩暥闁绘粠鍨冲ú浼村冀閸パ€鍋撻敓锟� 濠㈣埖鐗滈獮鍡欐喆閸屾稓銈撮柛濠忔嫹 闁告劕鎳愰獮鍡欐喆閸屾稓銈撮柛濠忔嫹  闁告劕鎳愰獮鍡欐喆閸屾稓銈撮柛濠勮嚕p闁挎稑濂旂粩鎾嚋椤掆偓閿濈偟鎷归悢鐑樼暠 err濠㈣泛瀚幃濠囧棘閻熸壆纭€
 *          err_cal_mode闁挎稒鐡眗r濠㈣泛瀚幃濠囧棘閻熸壆纭€ 闁告锕ゅ﹢鈧弶鈺偵戝Σ鎼佸炊濞戞ê鐎诲☉鏂款儎缁旀挳宕烽敓锟� 0闁挎冻鎷�1闁挎冻鎷�2 闂侇偆鍠庣€规娊鎮抽娆忊枏闁活澁鎷�0 yaw閺夌偟绻濇繛鍥偨閿燂拷1 
						闂傚嫧鍋撻柧鏄忔〃閸楀海鎲撮幒鎴濐唺闁绘粣鎷� 3
 *         闁告劕鎳愰獮鍡樼▔瀹ュ牆鍘村☉鎾剁樁ULL
 *	@note   濞达綀娉曢弫銈囩矆鏉炴壆浼愰柨娑虫嫹
			pid_ctrl_t *out	  = ;
			pid_ctrl_t *inn	  = ;
			float target   	  = ;
			float mea_out       = ;
			float mea_in        = ;
			float inner_kp      = ;
			uint8_t err_cal_mode= ;
			=all_pid_calc (out,inn,target,mea_out,mea_in,inner_kp,err_cal_mode);
 *  @author HERMIT_PURPLE
 *
 *  @return 閺夆晜鏌ㄥú鏍媼閿涘嫮鏆紓浣规尰閻忥拷
 */

float  all_pid_calc (pid_ctrl_t *out,pid_ctrl_t *inn,float target,float mea_out,float mea_in,float inner_kp,uint8_t err_cal_mode)
{
	if(inn == NULL)return 0;  //婵炲备鍓濆﹢渚€宕橀崨顖氱畾闁挎稑濂旂拹锟�0
	
	 else if(out == NULL&&inn!=NULL)  //闁告瑯浜濆﹢渚€鏌呴悢宄邦唺闁绘粣鎷�
	{
		inn->target=target;
		inn->measure=mea_in;
		inn->err=inn->target-mea_in;
		single_pid_ctrl(inn);
		return inn->out;
	}
	
	else if(out != NULL&&inn!=NULL)  //闁告瑥鐬奸獮鍝朓D
	{
		
		out->target=target;
		out->measure=mea_out;
		out->err=out->target-out->measure; //閻犱緤绱曢悾鑽ゆ喆閹烘垵顔婇柣婊庡灥椤曘倕顔忛鍡欑闁告艾閰ｅ浼村礃瀹ュ牏绠婚悶娑樼焷椤曘倕顔忛纰辨П闁荤儑鎷�
		switch(err_cal_mode)
		{
			
			case 0:			
				break;
			
			case 1:
				out->err = motor_half_cycle(out->err, 8191);
				break;		
			
			case 2:
				out->err = motor_half_cycle(out->err, 8191);
				out->err = motor_half_cycle(out->err, 4095);
				break;
			
			case 3:
				out->err = motor_half_cycle(out->err, 360);
				break;
			
			case 4:
				out->err = motor_half_cycle(out->err, 65535);
				break;
			
			case 5:
				out->err = motor_half_cycle(out->err, 191);
				break;
			default:
				break;
		}
		
		single_pid_ctrl(out);  //閻犱緤绱曢悾濠氬礄閸濆嫷妲遍柣鐐叉缁诲啰鎷犻姘枙闁汇劌瀚～妤佹償閿旀儳绠氶柣銊ュ閳ь剨鎷�
		inn->target=out->out; //閻熸瑦甯掔€规娊鎮抽婵堢炕闁告垼妗ㄧ紞鏃€绋夊ú顏佸亾閻斿嘲顔婇柣婊庡灣濞蹭即寮介崶褉鍋撻敓锟�
		inn->measure=mea_in*inner_kp;//闁告劕鎳愰獮鍡樻綇閹惧啿寮砶p闁挎稑鑻ぐ鍙夌閵夈劎娈堕柡浣哥摠椤掓粎鎷归悢閿嬪濠㈠爢鍐瘓
		inn->err=inn->target+inn->measure;  //闂侇偆鍠庣€规娊鎮抽婵愬殩鐎瑰壊鍠涢鍝ョ不閿燂拷
		single_pid_ctrl(inn);
		return inn->out;  //閺夊牊鎸搁崵顓㈠礃閸涱垰绠氶悹渚婄磿閻ｅ宕愰敓锟�
	}
	else  //闁告瑯浜濆﹢浣烘喆閹烘垵顔婇柣婊愭嫹
	{
		return 0;
	}
}


/**
 *	@brief   闁告挸绉归々鐠竔d閻犱緤绱曢悾锟�,濞达綀娉曢弫銈囩矆鏉炴壆浼愰柛娆忓€介埀顒€鍏巌d闁诡剛绮敮锟�
 *  @author HERMIT_PURPLE
 *  @return 閺夆晜鏌ㄥú鏍媼閿涘嫮鏆紓浣规尰閻忥拷
 */

float feedforward_pid_calc(float K_ff,pid_ctrl_t *out,pid_ctrl_t *inn,float target,float mea_out,float mea_in,float inner_kp,uint8_t err_cal_mode)
{
	float target_now;
	static float target_last;
	target_now=out->target;
	float output= (target_now-target_last)*K_ff +all_pid_calc (out,inn,target,mea_out,mea_in,inner_kp,err_cal_mode);
	target_last=target_now;
	return output;
}



