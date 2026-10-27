// Zero-dependency localhost server for Web Serial's secure context requirement.
const http=require('node:http');
const fs=require('node:fs');
const path=require('node:path');
const file=path.join(__dirname,'index.html');
http.createServer((req,res)=>{
  if(req.url!=='/'&&req.url!=='/index.html'){res.writeHead(404);res.end('Not found');return;}
  res.writeHead(200,{'Content-Type':'text/html; charset=utf-8','Cache-Control':'no-store'});
  fs.createReadStream(file).pipe(res);
}).listen(8765,'127.0.0.1',()=>console.log('Drone Flight Black Box: http://localhost:8765'));
