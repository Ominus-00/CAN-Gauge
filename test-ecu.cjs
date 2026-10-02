// Execute the compiled custom chip with a small Wokwi pin/timer test harness.
// Independently decode its emitted CAN bits using polynomial long division.
const fs = require('fs');
const assert = require('assert');
const mod = new WebAssembly.Module(fs.readFileSync(process.argv[2] || 'ecu.chip.wasm'));
const channelNames = ['IAT','BOOST','COOLANT','OIL_TEMP','EGT','OIL_P'];
const ranges = [[-40,120],[-100,300],[-40,140],[-40,180],[0,1100],[0,1000]];
function run(volts, acknowledge=false, enabled=true, busy=false) {
  const memory = new WebAssembly.Memory({initial:256});
  const u32=()=>new Uint32Array(memory.buffer);
  const names=[],modes=[],frames=[];
  let timer,next,inst,logs='',samples=[],sampleIndex=-1;
  const lengths=acknowledge?run(volts).frames.map(f=>f.length):[];
  const string=p=>{const a=new Uint8Array(memory.buffer);let q=p;while(a[q])q++;return Buffer.from(a.slice(p,q)).toString();};
  function pinRead(p){
    if(names[p]==='EN')return enabled?1:0;
    if((busy || (acknowledge && sampleIndex===lengths[frames.length]-12)) && /CAN[HL]/.test(names[p]))return names[p]==='CANH'?1:0;
    return modes[p]===17||modes[p]===2?1:0;
  }
  const env={memory,pinInit:(p,m)=>{names.push(string(p));modes.push(m);return names.length-1;},
    pinMode:(p,m)=>{modes[p]=m;},pinRead,
    pinADCRead:p=>volts[channelNames.indexOf(names[p])],
    timerInit:p=>{timer={data:u32()[p/4],fn:u32()[p/4+1]};return 1;},
    timerStart:(t,us)=>{next=us*1000;},timerStartNanos:(t,ns)=>{next=ns;}};
  const wasi={fd_close:()=>0,fd_seek:()=>0,fd_fdstat_get:()=>0,
    fd_write:(fd,p,n,out)=>{let total=0;for(let i=0;i<n;i++){let ptr=u32()[p/4+2*i],len=u32()[p/4+2*i+1];logs+=Buffer.from(new Uint8Array(memory.buffer,ptr,len)).toString();total+=len;}u32()[out/4]=total;return 0;}};
  inst=new WebAssembly.Instance(mod,{env,wasi_snapshot_preview1:wasi});
  inst.exports.chipInit();
  for(let k=0;k<3000;k++){
    const delay=next;next=undefined;
    if(delay===750000){sampleIndex=samples.length;samples.push(pinRead(names.indexOf('CANH'))&&!pinRead(names.indexOf('CANL'))?0:1);}
    else sampleIndex=-1;
    inst.exports.__indirect_function_table.get(timer.fn)(timer.data);
    if(samples.length && next===100000000){frames.push(samples);samples=[];if(frames.length===6)break;}
    if(!enabled && k>1)break;
  }
  return {frames,logs};
}
function decode(wire){
  let raw=[],p=0,last=-1,run=0;
  while(raw.length<50){let b=wire[p++];assert(b===0||b===1);raw.push(b);run=b===last?run+1:1;last=b;
    if(run===5){const stuffed=wire[p++];assert.equal(stuffed,1-b,'stuff bit');last=stuffed;run=1;}}
  const field=(a,n)=>raw.slice(a,a+n).reduce((v,b)=>v*2+b,0);
  assert.equal(field(0,1),0);assert.equal(field(12,3),0);assert.equal(field(15,4),2);
  const rem=[...raw],poly=0xc599;
  for(let i=0;i<35;i++)if(rem[i])for(let j=0;j<16;j++)rem[i+j]^=(poly>>(15-j))&1;
  assert(rem.slice(-15).every(b=>b===0),'CRC remainder');
  assert.equal(wire.length-p,13);assert.equal(wire[p],1);assert(wire.slice(p+2).every(b=>b===1));
  let value=field(19,16);if(value>=32768)value-=65536;
  return {id:field(1,11),value};
}
let cases=0;
function verify(volts){
  const result=run(volts,true);assert.equal(result.frames.length,6);
  assert(result.frames.every(bits => bits[bits.length-12] === 0), 'ACK on every frame');
  result.frames.forEach((bits,i)=>{
    let v=volts[i];if(!(v>=0))v=0;v=Math.min(5,v);
    let value=ranges[i][0]+v*(ranges[i][1]-ranges[i][0])/5;
    value=value<0?-Math.floor(-value+0.5):Math.floor(value+0.5);
    assert.deepEqual(decode(bits),{id:0x123+i,value:value===0?0:value});
  });++cases;
}
for(const v of [0,2.5,5,-1,6,NaN])verify(Array(6).fill(v));
for(let i=0;i<6;i++)for(const v of [0,1.25,3.75,5]){let a=Array(6).fill(2.5);a[i]=v;verify(a);}
assert.equal(run(Array(6).fill(2.5),false,false).frames.length,0);
assert.equal(run(Array(6).fill(2.5),false,true,true).frames.length,0);
assert(run(Array(6).fill(2.5)).logs.includes('ACK=NO'));
console.log(`PASS: ${cases} six-channel cases (${cases*6} CAN frames), independent inputs, signed ranges, IDs, stuffing, CRC, ACK, EN and busy bus.`);

