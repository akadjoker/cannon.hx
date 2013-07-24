package io.schteppe.cannon.utils;

/**
 * @class CANNON.EventTarget
 * @see https://github.com/mrdoob/eventtarget.js/
 */
class EventTarget {

    var listeners:Map<String, Array<Dynamic>>;

    public function new() {
        listeners = new Map<String, Array<Dynamic>>();
    }

    public function addEventListener ( type:String, listener:Dynamic = null) {
        if ( listeners[ type ] == null ) {
            listeners[ type ] = [];
        }
        if (Lambda.indexOf(listeners[ type ], listener) == -1 ) {
            listeners[ type ].push( listener );
        }
    }
    public function dispatchEvent ( event:Dynamic ) {
        var type:String = event.type;
        var listenersArray:Array<Dynamic> = listeners[ type ];
        if (listenersArray != null) {
            for ( listener in listenersArray ) {
                listener( event );
            }
        }
    }
    public function removeEventListener ( type, listener:Dynamic ) {
        var index = Lambda.indexOf(listeners[ type ], listener);
        if ( index != - 1 ) {
            listeners[ type ].splice( index, 1 );
        }
    }
}
