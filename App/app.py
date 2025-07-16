import streamlit as st
from PIL import Image
from yolo_utils import process_full_output

st.set_page_config(layout="wide")
st.title("🧠 FOD Detection & Size Estimation")

uploaded_file = st.file_uploader("📤 Upload an image", type=["jpg", "jpeg", "png"])

if uploaded_file is not None:
    image = Image.open(uploaded_file).convert("RGB")
    original_img, bbox_img, rotated_img, log = process_full_output(image)

    col1, col2, col3 = st.columns(3)
    with col1:
        st.subheader("🖼️ Original Image")
        st.image(original_img, use_container_width=True)

    with col2:
        st.subheader("📦 YOLO Bounding Box")
        st.image(bbox_img, use_container_width=True)

    with col3:
        st.subheader("📐 Rotated Bounding Box")
        st.image(rotated_img, use_container_width=True)

    st.subheader("📏 Measurement Results")
    st.text_area("Detected Sizes (mm)", log, height=200)
