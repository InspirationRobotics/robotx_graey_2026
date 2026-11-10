
(function () {
  var t = null;
  try { t = localStorage.getItem("crsd-theme"); } catch (e) {}
  if (t !== "light" && t !== "dark") t = matchMedia("(prefers-color-scheme: dark)").matches ? "dark" : "light";
  document.documentElement.setAttribute("data-theme", t);
})();

addEventListener('storage',e=>{if(e.key==='crsd-theme'&&['light','dark'].includes(e.newValue))document.documentElement.dataset.theme=e.newValue});
addEventListener('message',e=>{if(e.origin===location.origin&&['light','dark'].includes(e.data?.graeyTheme))document.documentElement.dataset.theme=e.data.graeyTheme});
