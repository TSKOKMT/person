uniform sampler2DRect inputTexture;
uniform int redCard;
uniform int greenCard;
uniform int blueCard;

void main() {
    vec4 inputColor = texture2DRect(inputTexture, gl_TexCoord[0].xy);
    gl_FragColor = vec4(inputColor[redCard], inputColor[greenCard], inputColor[blueCard], inputColor.a);
}
