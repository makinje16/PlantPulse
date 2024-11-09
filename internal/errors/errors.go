package errors

import "errors"

var ErrUnimplemented = errors.New("ErrUnimplemented: method isn't implemented")
var ErrUnsupportedMessage = errors.New("ErrUnsupportedMessage: message isn't supported")
