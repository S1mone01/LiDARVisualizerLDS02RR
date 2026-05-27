package org.lidar;

import android.webkit.WebView;
import android.webkit.WebViewClient;
import android.webkit.WebResourceRequest;

public class CustomWebViewClient extends WebViewClient {
    public interface Callback {
        void onConnectUsb();
    }
    
    private Callback callback;
    
    public CustomWebViewClient(Callback callback) {
        this.callback = callback;
    }

    @Override
    public boolean shouldOverrideUrlLoading(WebView view, String url) {
        if (url != null && url.startsWith("app://connect_usb")) {
            if (this.callback != null) {
                this.callback.onConnectUsb();
            }
            return true;
        }
        return false;
    }
    
    @Override
    public boolean shouldOverrideUrlLoading(WebView view, WebResourceRequest request) {
        if (request != null && request.getUrl() != null) {
            String url = request.getUrl().toString();
            if (url.startsWith("app://connect_usb")) {
                if (this.callback != null) {
                    this.callback.onConnectUsb();
                }
                return true;
            }
        }
        return false;
    }
}
