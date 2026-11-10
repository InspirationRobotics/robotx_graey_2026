// Browser preference only; never changes vehicle state.
(function(){
  function apply(){try{document.documentElement.dataset.theme=localStorage.getItem('graeyTheme')||'dark'}catch(e){}}
  apply();window.addEventListener('storage',apply);
  if(window.parent!==window)document.documentElement.classList.add('embedded-station');
})();
