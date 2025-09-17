// LOGGING SYSTEM
// ==================================================================================
class Logger {
    constructor() {
        this.logContent = document.getElementById('log-content');
        this.logToggle = document.getElementById('log-toggle');
        this.logClear = document.getElementById('log-clear');
        this.isCollapsed = false;
        
        this.logToggle.addEventListener('click', () => {
            this.toggleCollapse();
        });
        
        this.logClear.addEventListener('click', () => {
            this.clear();
        });
    }
    
    toggleCollapse() {
        this.isCollapsed = !this.isCollapsed;
        this.logContent.classList.toggle('collapsed', this.isCollapsed);
        this.logToggle.textContent = this.isCollapsed ? '+' : '−';
    }
    
    clear() {
        this.logContent.innerHTML = '';
    }
    
    log(message, color = 'wheat') {
        const entry = document.createElement('div');
        entry.className = `log-entry log-${color}`;
        // entry.textContent = `[${new Date().toLocaleTimeString()}] ${message}`;
        entry.textContent = `• ${message}`;
        this.logContent.appendChild(entry);
        this.logContent.scrollTop = this.logContent.scrollHeight;
        if (this.logContent.children.length > 100) {
            this.logContent.removeChild(this.logContent.firstChild);
        }
    }
    
    green(message) { this.log(message, 'green'); }
    red(message) { this.log(message, 'red'); }
    yellow(message) { this.log(message, 'yellow'); }
    normal(message) { this.log(message, 'wheat'); }
}