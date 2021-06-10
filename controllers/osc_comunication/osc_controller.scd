// retrieve the current IP and port
NetAddr.localAddr

OSCFunc.trace(true); // Turn posting on
OSCFunc.trace(false); // Turn posting off

s.boot;

(
SynthDef(\pmCrotales, {
	arg out = 0, freq = 261, tone = 3, att = 0, rel = 2, curve = -6, amp = 0.8, pan = 0, modLo = 5.25, modHi = 5.5;

	var env, snd, mod;

	env = Env.perc(attackTime: 0, releaseTime: rel, curve: curve).kr(doneAction: 2);

	mod = Rand(modLo, modHi);

	snd = PMOsc.ar(
	    	carfreq: freq,
	    	modfreq: mod * freq,
	    	pmindex: env * tone,
	    	mul: env * amp
	    );

	snd = HPF.ar(snd, freq / 2);

	snd = Mix.ar(snd) * 0.1;

	Out.ar(out, Pan2.ar(snd, pan));
};
).add;

//receiving OSC in SC from Webots
OSCdef.new(\makeping, {
	arg msg;
	if( msg[1] > 0.01, {
		Synth(\pmCrotales, [
			\octave, 10,
			\scale, msg[1],
			\degree,rrand(0, 1),
			\amp, 0.3,
			\pan, rrand(-0.5, 0.5),
			\dur, 0.25
		]);
	});
}, "/controllers/osc_comunication");

)

OSCdef.freeAll;

//gravar o áudio gerado direto do servidor
s.makeWindow