from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="shoulder_camcorder")

    # ---- Materials ---------------------------------------------------------
    body_mat = model.material("body_shell", rgba=(0.10, 0.10, 0.11, 1.0))
    accent_mat = model.material("accent", rgba=(0.18, 0.18, 0.20, 1.0))
    rubber_mat = model.material("rubber", rgba=(0.05, 0.05, 0.06, 1.0))
    lens_mat = model.material("lens_barrel", rgba=(0.12, 0.12, 0.13, 1.0))
    glass_mat = model.material("lens_glass", rgba=(0.10, 0.18, 0.28, 1.0))
    ring_mat = model.material("ring", rgba=(0.22, 0.22, 0.24, 1.0))
    screen_back_mat = model.material("screen_back", rgba=(0.08, 0.08, 0.10, 1.0))
    screen_glass_mat = model.material("screen_glass", rgba=(0.30, 0.55, 0.80, 1.0))
    red_mat = model.material("record_red", rgba=(0.78, 0.10, 0.10, 1.0))
    silver_mat = model.material("silver", rgba=(0.55, 0.55, 0.58, 1.0))

    # ---- Body --------------------------------------------------------------
    BL, BW, BH = 0.22, 0.12, 0.13  # length(x), width(y), height(z)

    body = model.part("body")
    body.visual(
        Box((BL, BW, BH)),
        origin=Origin(xyz=(0.0, 0.0, BH / 2)),
        material=body_mat,
        name="body_shell",
    )
    # Top accent plate
    body.visual(
        Box((BL * 0.86, BW * 0.94, 0.012)),
        origin=Origin(xyz=(-0.005, 0.0, BH + 0.006)),
        material=accent_mat,
        name="top_plate",
    )
    # Shoulder pad on the bottom
    body.visual(
        Box((BL * 0.72, BW * 1.02, 0.018)),
        origin=Origin(xyz=(-0.015, 0.0, -0.009)),
        material=rubber_mat,
        name="shoulder_pad",
    )
    # Front lens-mount collar
    body.visual(
        Cylinder(radius=0.054, length=0.014),
        origin=Origin(xyz=(BL / 2 + 0.007, 0.0, BH / 2), rpy=(0, math.pi / 2, 0)),
        material=accent_mat,
        name="lens_mount_collar",
    )

    # ---- Carry handle ------------------------------------------------------
    handle_base_z = BH + 0.012
    support_h = 0.052
    handle_top_z = handle_base_z + support_h
    front_sup_x = BL / 2 - 0.045
    rear_sup_x = -BL / 2 + 0.060
    body.visual(
        Box((0.024, 0.030, support_h)),
        origin=Origin(xyz=(front_sup_x, 0.0, handle_base_z + support_h / 2)),
        material=body_mat,
        name="handle_front_post",
    )
    body.visual(
        Box((0.024, 0.030, support_h)),
        origin=Origin(xyz=(rear_sup_x, 0.0, handle_base_z + support_h / 2)),
        material=body_mat,
        name="handle_rear_post",
    )
    bar_len = front_sup_x - rear_sup_x + 0.024
    bar_x = (front_sup_x + rear_sup_x) / 2
    body.visual(
        Box((bar_len, 0.034, 0.022)),
        origin=Origin(xyz=(bar_x, 0.0, handle_top_z + 0.011)),
        material=body_mat,
        name="handle_top_bar",
    )
    # Rubber grip wrap on top bar (overlaps top of bar so it stays connected)
    body.visual(
        Box((bar_len * 0.65, 0.036, 0.012)),
        origin=Origin(xyz=(bar_x, 0.0, handle_top_z + 0.022 + 0.005)),
        material=rubber_mat,
        name="handle_grip",
    )

    # ---- Eyepiece (rear) ---------------------------------------------------
    eye_y = 0.030
    eye_z = BH * 0.78
    body.visual(
        Cylinder(radius=0.020, length=0.052),
        origin=Origin(xyz=(-BL / 2 - 0.024, eye_y, eye_z), rpy=(0, math.pi / 2, 0)),
        material=accent_mat,
        name="eyepiece_tube",
    )
    body.visual(
        Cylinder(radius=0.028, length=0.020),
        origin=Origin(xyz=(-BL / 2 - 0.058, eye_y, eye_z), rpy=(0, math.pi / 2, 0)),
        material=rubber_mat,
        name="eyecup",
    )
    body.visual(
        Cylinder(radius=0.022, length=0.008),
        origin=Origin(xyz=(-BL / 2 - 0.005, eye_y, eye_z), rpy=(0, math.pi / 2, 0)),
        material=ring_mat,
        name="diopter_ring",
    )

    # ---- LCD hinge mounts on body's left side ------------------------------
    LCD_HINGE_X = 0.018
    LCD_HINGE_Y_AXIS = -BW / 2 - 0.014
    LCD_HINGE_Z_BOT = BH * 0.20
    LCD_HINGE_Z_TOP = BH * 0.85
    LCD_HINGE_Z_MID = (LCD_HINGE_Z_BOT + LCD_HINGE_Z_TOP) / 2
    LCD_HINGE_LEN = LCD_HINGE_Z_TOP - LCD_HINGE_Z_BOT

    body.visual(
        Cylinder(radius=0.0065, length=LCD_HINGE_LEN),
        origin=Origin(xyz=(LCD_HINGE_X, LCD_HINGE_Y_AXIS, LCD_HINGE_Z_MID)),
        material=accent_mat,
        name="lcd_hinge_barrel_body",
    )
    # Bracket lugs that bridge from body face to barrel (top + bottom).
    # Make them long enough in y to bridge between body face (-0.06) and barrel center (-0.069).
    body.visual(
        Box((0.020, 0.020, 0.014)),
        origin=Origin(xyz=(LCD_HINGE_X, -BW / 2 - 0.008, LCD_HINGE_Z_TOP - 0.007)),
        material=accent_mat,
        name="lcd_hinge_lug_top",
    )
    body.visual(
        Box((0.020, 0.020, 0.014)),
        origin=Origin(xyz=(LCD_HINGE_X, -BW / 2 - 0.008, LCD_HINGE_Z_BOT + 0.007)),
        material=accent_mat,
        name="lcd_hinge_lug_bot",
    )
    # LCD bay recess plate on body left face
    body.visual(
        Box((0.115, 0.003, 0.075)),
        origin=Origin(xyz=(0.025, -BW / 2 - 0.0015, BH * 0.52)),
        material=accent_mat,
        name="lcd_bay_plate",
    )

    # ---- Media door bay frame on body's right side -------------------------
    DOOR_X_C = -0.030
    DOOR_LEN = 0.090
    DOOR_HEIGHT = 0.072
    DOOR_THICK = 0.006
    body.visual(
        Box((DOOR_LEN + 0.010, 0.003, DOOR_HEIGHT + 0.010)),
        origin=Origin(xyz=(DOOR_X_C, BW / 2 + 0.0015, BH * 0.46)),
        material=accent_mat,
        name="media_bay_frame",
    )
    body.visual(
        Cylinder(radius=0.0045, length=DOOR_HEIGHT),
        origin=Origin(
            xyz=(DOOR_X_C - DOOR_LEN / 2 - 0.003, BW / 2 + 0.005, BH * 0.46)
        ),
        material=accent_mat,
        name="media_door_barrel_body",
    )
    # Bracket lug to bridge body face to door barrel
    body.visual(
        Box((0.008, 0.014, DOOR_HEIGHT * 0.95)),
        origin=Origin(
            xyz=(DOOR_X_C - DOOR_LEN / 2 - 0.005, BW / 2 + 0.004, BH * 0.46)
        ),
        material=accent_mat,
        name="media_door_bracket",
    )

    # ---- Lens barrel (fixed sections — contiguous along X) -----------------
    LENS_R = 0.046
    LENS_LEN = 0.165
    LENS_X_START = BL / 2 + 0.014
    LENS_CZ = BH / 2

    # Section breakpoints (fractions of LENS_LEN): rear 0..0.45, mid 0.45..0.65,
    # front 0.65..0.95, hood 0.90..1.00. Sections share boundary planes so the
    # body remains a single connected component.
    body.visual(
        Cylinder(radius=LENS_R, length=LENS_LEN * 0.45),
        origin=Origin(
            xyz=(LENS_X_START + LENS_LEN * 0.225, 0.0, LENS_CZ),
            rpy=(0, math.pi / 2, 0),
        ),
        material=lens_mat,
        name="lens_barrel_rear",
    )
    body.visual(
        Cylinder(radius=LENS_R - 0.006, length=LENS_LEN * 0.20),
        origin=Origin(
            xyz=(LENS_X_START + LENS_LEN * 0.55, 0.0, LENS_CZ),
            rpy=(0, math.pi / 2, 0),
        ),
        material=lens_mat,
        name="lens_barrel_mid",
    )
    body.visual(
        Cylinder(radius=LENS_R, length=LENS_LEN * 0.30),
        origin=Origin(
            xyz=(LENS_X_START + LENS_LEN * 0.80, 0.0, LENS_CZ),
            rpy=(0, math.pi / 2, 0),
        ),
        material=lens_mat,
        name="lens_barrel_front",
    )
    body.visual(
        Cylinder(radius=LENS_R + 0.006, length=LENS_LEN * 0.12),
        origin=Origin(
            xyz=(LENS_X_START + LENS_LEN * 0.94, 0.0, LENS_CZ),
            rpy=(0, math.pi / 2, 0),
        ),
        material=lens_mat,
        name="lens_hood",
    )
    body.visual(
        Cylinder(radius=LENS_R - 0.005, length=0.004),
        origin=Origin(
            xyz=(LENS_X_START + LENS_LEN - 0.003, 0.0, LENS_CZ),
            rpy=(0, math.pi / 2, 0),
        ),
        material=glass_mat,
        name="lens_glass",
    )

    # ---- Power switch (fixed accent on rear) -------------------------------
    body.visual(
        Box((0.014, 0.022, 0.010)),
        origin=Origin(xyz=(-BL / 2 - 0.005, -0.030, BH * 0.30)),
        material=accent_mat,
        name="power_switch_housing",
    )
    body.visual(
        Box((0.006, 0.012, 0.006)),
        origin=Origin(xyz=(-BL / 2 - 0.011, -0.030, BH * 0.30)),
        material=silver_mat,
        name="power_switch_lever",
    )

    # ---- Zoom-rocker mounting cradle on top --------------------------------
    ZOOM_X = 0.045
    ZOOM_Y = BW / 2 - 0.028
    ZOOM_Z_TOP = BH + 0.012  # surface of top plate
    body.visual(
        Box((0.044, 0.030, 0.010)),
        origin=Origin(xyz=(ZOOM_X, ZOOM_Y, ZOOM_Z_TOP + 0.005)),
        material=accent_mat,
        name="zoom_cradle",
    )

    # ---- Record-button bezel on rear ---------------------------------------
    REC_X = -BL / 2
    REC_Y = 0.026
    REC_Z = BH * 0.46
    body.visual(
        Cylinder(radius=0.011, length=0.006),
        origin=Origin(
            xyz=(REC_X - 0.002, REC_Y, REC_Z), rpy=(0, math.pi / 2, 0)
        ),
        material=accent_mat,
        name="record_bezel",
    )

    # ========================================================================
    # ROTATING LENS RINGS
    # ========================================================================
    # Zoom ring — sits over the rear lens barrel section.
    zoom_ring = model.part("zoom_ring")
    ZRING_LEN = 0.024
    ZRING_R = LENS_R + 0.003
    zoom_ring.visual(
        Cylinder(radius=ZRING_R, length=ZRING_LEN),
        origin=Origin(xyz=(0, 0, 0), rpy=(0, math.pi / 2, 0)),
        material=ring_mat,
        name="zoom_ring_grip",
    )
    ZRING_X = LENS_X_START + LENS_LEN * 0.22
    model.articulation(
        "zoom_ring_pivot",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=zoom_ring,
        origin=Origin(xyz=(ZRING_X, 0.0, LENS_CZ)),
        axis=(1, 0, 0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0),
    )

    # Focus ring — sits over the front lens barrel section, well away from
    # both the mid section and the hood.
    focus_ring = model.part("focus_ring")
    FOCUS_LEN = 0.026
    FOCUS_R = LENS_R + 0.004
    focus_ring.visual(
        Cylinder(radius=FOCUS_R, length=FOCUS_LEN),
        origin=Origin(xyz=(0, 0, 0), rpy=(0, math.pi / 2, 0)),
        material=ring_mat,
        name="focus_ring_grip",
    )
    focus_ring.visual(
        Cylinder(radius=FOCUS_R + 0.001, length=FOCUS_LEN * 0.4),
        origin=Origin(xyz=(0, 0, 0), rpy=(0, math.pi / 2, 0)),
        material=accent_mat,
        name="focus_ring_band",
    )
    FOCUS_X = LENS_X_START + LENS_LEN * 0.78
    model.articulation(
        "focus_ring_pivot",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=focus_ring,
        origin=Origin(xyz=(FOCUS_X, 0.0, LENS_CZ)),
        axis=(1, 0, 0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0),
    )

    # ========================================================================
    # LCD ARM (swings out from body left side around vertical axis)
    # ========================================================================
    ARM_L = 0.082
    ARM_T = 0.014
    ARM_H = 0.062

    lcd_arm = model.part("lcd_arm")
    lcd_arm.visual(
        Cylinder(radius=0.0085, length=LCD_HINGE_LEN * 0.80),
        origin=Origin(xyz=(0, 0, 0)),
        material=accent_mat,
        name="lcd_arm_barrel",
    )
    lcd_arm.visual(
        Box((ARM_L, ARM_T, ARM_H)),
        origin=Origin(xyz=(0.005 + ARM_L / 2, 0.0, 0.0)),
        material=body_mat,
        name="lcd_arm_body",
    )
    DISTAL_BARREL_R = 0.011
    DISTAL_BARREL_LEN = 0.030
    DISTAL_X = 0.005 + ARM_L + DISTAL_BARREL_LEN * 0.25
    lcd_arm.visual(
        Cylinder(radius=DISTAL_BARREL_R, length=DISTAL_BARREL_LEN),
        origin=Origin(xyz=(DISTAL_X, 0.0, 0.0), rpy=(0, math.pi / 2, 0)),
        material=accent_mat,
        name="lcd_distal_barrel",
    )

    model.articulation(
        "body_to_lcd_arm",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lcd_arm,
        origin=Origin(xyz=(LCD_HINGE_X, LCD_HINGE_Y_AXIS, LCD_HINGE_Z_MID)),
        axis=(0, 0, -1),
        motion_limits=MotionLimits(
            effort=2.0, velocity=2.0, lower=0.0, upper=math.pi * 0.55
        ),
    )

    # ========================================================================
    # LCD PANEL (rotates on the arm around the arm's long axis)
    # ========================================================================
    PANEL_W = 0.072
    PANEL_T = 0.009
    PANEL_H = 0.050

    lcd_panel = model.part("lcd_panel")
    # Move the panel further down so it clears the arm body box at q=0.
    panel_z_c = -DISTAL_BARREL_R - 0.026 - PANEL_H / 2  # ~-0.062
    # Collar that wraps the distal pivot barrel — visible support tying the
    # panel to the arm.
    lcd_panel.visual(
        Cylinder(radius=DISTAL_BARREL_R + 0.003, length=DISTAL_BARREL_LEN * 0.6),
        origin=Origin(xyz=(0, 0, 0), rpy=(0, math.pi / 2, 0)),
        material=accent_mat,
        name="lcd_panel_collar",
    )
    # Stem from collar down to panel back. Slightly overlaps both ends.
    stem_z_top = -DISTAL_BARREL_R + 0.001
    stem_z_bot = panel_z_c + PANEL_H / 2 - 0.001
    stem_len = stem_z_top - stem_z_bot
    lcd_panel.visual(
        Box((0.014, 0.010, stem_len)),
        origin=Origin(xyz=(0, 0, (stem_z_top + stem_z_bot) / 2)),
        material=accent_mat,
        name="lcd_panel_stem",
    )
    # Panel back (housing)
    lcd_panel.visual(
        Box((PANEL_W, PANEL_T, PANEL_H)),
        origin=Origin(xyz=(0, 0, panel_z_c)),
        material=screen_back_mat,
        name="lcd_panel_back",
    )
    # Active screen face on -Y side of panel back (overlapping back slightly)
    lcd_panel.visual(
        Box((PANEL_W * 0.86, 0.0015, PANEL_H * 0.80)),
        origin=Origin(xyz=(0, -PANEL_T / 2 - 0.0001, panel_z_c)),
        material=screen_glass_mat,
        name="lcd_panel_screen",
    )

    model.articulation(
        "lcd_arm_to_panel",
        ArticulationType.REVOLUTE,
        parent=lcd_arm,
        child=lcd_panel,
        origin=Origin(xyz=(DISTAL_X, 0.0, 0.0)),
        axis=(1, 0, 0),
        motion_limits=MotionLimits(
            effort=1.0, velocity=2.0, lower=-math.pi, upper=math.pi
        ),
    )

    # ========================================================================
    # MEDIA DOOR (hinged on body right side)
    # ========================================================================
    media_door = model.part("media_door")
    media_door.visual(
        Cylinder(radius=0.0050, length=DOOR_HEIGHT * 0.95),
        origin=Origin(xyz=(0, 0, 0)),
        material=accent_mat,
        name="media_door_barrel",
    )
    # Door panel extends along +X from the hinge axis, sitting on the -Y side
    # of the hinge (against body face when closed).
    media_door.visual(
        Box((DOOR_LEN, DOOR_THICK, DOOR_HEIGHT)),
        origin=Origin(xyz=(DOOR_LEN / 2 + 0.004, -DOOR_THICK / 2 - 0.001, 0)),
        material=body_mat,
        name="media_door_panel",
    )
    # Latch nub (overlaps the panel's inner face slightly so it stays connected)
    media_door.visual(
        Box((0.006, 0.005, 0.014)),
        origin=Origin(xyz=(DOOR_LEN - 0.001, -DOOR_THICK - 0.0005, 0)),
        material=accent_mat,
        name="media_door_latch",
    )
    model.articulation(
        "body_to_media_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=media_door,
        origin=Origin(
            xyz=(
                DOOR_X_C - DOOR_LEN / 2 - 0.003,
                BW / 2 + 0.005,
                BH * 0.46,
            )
        ),
        axis=(0, 0, 1),
        motion_limits=MotionLimits(
            effort=1.0, velocity=2.0, lower=0.0, upper=math.pi * 0.6
        ),
    )

    # ========================================================================
    # RECORD BUTTON (prismatic press, on rear face)
    # ========================================================================
    record_button = model.part("record_button")
    # Cylinder along X (press direction). At q=0 cap protrudes -X from rear face.
    record_button.visual(
        Cylinder(radius=0.009, length=0.006),
        origin=Origin(xyz=(-0.003, 0.0, 0.0), rpy=(0, math.pi / 2, 0)),
        material=red_mat,
        name="record_button_cap",
    )
    model.articulation(
        "record_button_press",
        ArticulationType.PRISMATIC,
        parent=body,
        child=record_button,
        origin=Origin(xyz=(REC_X, REC_Y, REC_Z)),
        axis=(1, 0, 0),
        motion_limits=MotionLimits(
            effort=1.0, velocity=0.05, lower=0.0, upper=0.003
        ),
    )

    # ========================================================================
    # ZOOM ROCKER (revolute rocker on top)
    # ========================================================================
    zoom_rocker = model.part("zoom_rocker")
    zoom_rocker.visual(
        Box((0.034, 0.020, 0.006)),
        origin=Origin(xyz=(0, 0, 0.003)),
        material=silver_mat,
        name="zoom_rocker_body",
    )
    zoom_rocker.visual(
        Box((0.005, 0.018, 0.008)),
        origin=Origin(xyz=(0.014, 0, 0.005)),
        material=accent_mat,
        name="zoom_rocker_cap_t",
    )
    zoom_rocker.visual(
        Box((0.005, 0.018, 0.008)),
        origin=Origin(xyz=(-0.014, 0, 0.005)),
        material=accent_mat,
        name="zoom_rocker_cap_w",
    )
    # Pivot pin — visible cylinder along Y bridging the rocker into the cradle
    zoom_rocker.visual(
        Cylinder(radius=0.0035, length=0.030),
        origin=Origin(xyz=(0, 0, 0), rpy=(math.pi / 2, 0, 0)),
        material=silver_mat,
        name="zoom_rocker_pin",
    )
    model.articulation(
        "zoom_rocker_pivot",
        ArticulationType.REVOLUTE,
        parent=body,
        child=zoom_rocker,
        origin=Origin(xyz=(ZOOM_X, ZOOM_Y, ZOOM_Z_TOP + 0.010)),
        axis=(0, 1, 0),
        motion_limits=MotionLimits(
            effort=0.5, velocity=2.0, lower=-0.20, upper=0.20
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    lcd_arm = object_model.get_part("lcd_arm")
    lcd_panel = object_model.get_part("lcd_panel")
    media_door = object_model.get_part("media_door")
    focus_ring = object_model.get_part("focus_ring")
    zoom_ring = object_model.get_part("zoom_ring")
    record_button = object_model.get_part("record_button")
    zoom_rocker = object_model.get_part("zoom_rocker")

    arm_hinge = object_model.get_articulation("body_to_lcd_arm")
    door_hinge = object_model.get_articulation("body_to_media_door")
    record_press = object_model.get_articulation("record_button_press")

    # ------------------------------------------------------------------
    # Intentional overlap allowances: control rings encircle their lens
    # barrel sections; hinge pins/barrels are concentric inside captured
    # bracket lugs.
    # ------------------------------------------------------------------
    ctx.allow_overlap(
        body, focus_ring,
        elem_a="lens_barrel_front", elem_b="focus_ring_grip",
        reason="Focus control ring encircles the lens barrel front section.",
    )
    ctx.allow_overlap(
        body, focus_ring,
        elem_a="lens_barrel_front", elem_b="focus_ring_band",
        reason="Focus control ring encircles the lens barrel front section.",
    )
    ctx.allow_overlap(
        body, zoom_ring,
        elem_a="lens_barrel_rear", elem_b="zoom_ring_grip",
        reason="Zoom control ring encircles the lens barrel rear section.",
    )
    ctx.allow_overlap(
        lcd_arm, lcd_panel,
        elem_a="lcd_distal_barrel", elem_b="lcd_panel_collar",
        reason="Panel collar captures the arm's distal pivot barrel.",
    )
    ctx.allow_overlap(
        body, lcd_arm,
        elem_a="lcd_hinge_barrel_body", elem_b="lcd_arm_barrel",
        reason="Arm barrel and body barrel are concentric on the hinge axis.",
    )
    ctx.allow_overlap(
        body, lcd_arm,
        elem_a="lcd_hinge_lug_top", elem_b="lcd_arm_barrel",
        reason="Top hinge lug captures the arm barrel.",
    )
    ctx.allow_overlap(
        body, lcd_arm,
        elem_a="lcd_hinge_lug_bot", elem_b="lcd_arm_barrel",
        reason="Bottom hinge lug captures the arm barrel.",
    )
    ctx.allow_overlap(
        body, media_door,
        elem_a="media_door_barrel_body", elem_b="media_door_barrel",
        reason="Door barrel sits concentric with the body hinge barrel.",
    )
    ctx.allow_overlap(
        body, media_door,
        elem_a="media_door_bracket", elem_b="media_door_barrel",
        reason="Door barrel is captured inside the body-side bracket lug.",
    )
    ctx.allow_overlap(
        body, zoom_rocker,
        elem_a="zoom_cradle", elem_b="zoom_rocker_pin",
        reason="Zoom rocker pivot pin is captured in the cradle.",
    )
    ctx.allow_overlap(
        body, record_button,
        elem_a="record_bezel", elem_b="record_button_cap",
        reason="Record button cap protrudes through the bezel ring.",
    )

    # ------------------------------------------------------------------
    # Exact rest-pose assertions (element-scoped where part AABBs would
    # otherwise pick up unrelated features like hinge barrels).
    # ------------------------------------------------------------------
    # LCD arm body sits clear of body shell on -Y at rest.
    ctx.expect_gap(
        body, lcd_arm,
        axis="y",
        positive_elem="body_shell",
        negative_elem="lcd_arm_body",
        max_penetration=0.0,
        max_gap=0.010,
        name="lcd arm body clears body left face at rest",
    )

    # Distal barrel of the arm captures the panel collar (concentric retention).
    ctx.expect_within(
        lcd_panel, lcd_arm,
        axes="x",
        inner_elem="lcd_panel_collar", outer_elem="lcd_distal_barrel",
        margin=0.001,
        name="panel collar stays centered on the arm distal barrel",
    )

    # Record button cap protrudes behind body at rest.
    ctx.expect_gap(
        body, record_button,
        axis="x",
        positive_elem="body_shell", negative_elem="record_button_cap",
        max_penetration=0.001,
        name="record button cap protrudes from rear at rest",
    )

    # Zoom rocker rests on top of cradle.
    ctx.expect_gap(
        zoom_rocker, body,
        axis="z",
        positive_elem="zoom_rocker_body", negative_elem="zoom_cradle",
        max_gap=0.005,
        max_penetration=0.0,
        name="zoom rocker sits on its cradle",
    )

    # ------------------------------------------------------------------
    # Decisive pose checks for the primary mechanisms.
    # ------------------------------------------------------------------
    # LCD arm at upper limit swings clear of body face.
    with ctx.pose({arm_hinge: math.pi * 0.55}):
        ctx.expect_gap(
            body, lcd_arm,
            axis="y",
            positive_elem="body_shell", negative_elem="lcd_arm_body",
            min_gap=0.010,
            name="opened lcd arm body clears body face in -Y",
        )

    # Media door swings out at upper limit.
    with ctx.pose({door_hinge: math.pi * 0.5}):
        ctx.expect_gap(
            media_door, body,
            axis="y",
            positive_elem="media_door_panel", negative_elem="body_shell",
            min_gap=0.005,
            name="opened media door panel clears body in +Y",
        )

    # Record button presses inward when actuated.
    rest_cap_x = ctx.part_world_position(record_button)
    with ctx.pose({record_press: 0.003}):
        pressed_cap_x = ctx.part_world_position(record_button)
    ctx.check(
        "record button moves inward when pressed",
        rest_cap_x is not None and pressed_cap_x is not None
        and pressed_cap_x[0] - rest_cap_x[0] > 0.002,
        details=f"rest={rest_cap_x}, pressed={pressed_cap_x}",
    )

    return ctx.report()


object_model = build_object_model()
