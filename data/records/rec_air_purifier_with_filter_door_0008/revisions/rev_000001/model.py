from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BODY_W = 0.62
BODY_H = 0.84
BODY_D = 0.18
BACK_T = 0.014
SHELL_T = 0.032
FRONT_BEZEL_T = 0.011
BEZEL_Z = 0.156
HINGE_RADIUS = 0.010
HINGE_SEG_LEN = 0.064
HINGE_PITCH = 0.070
LATCH_X = 0.182
DOOR_W = 0.56
DOOR_H = 0.72
DOOR_T = 0.026
HINGE_AXIS_Y = -0.35
HINGE_AXIS_Z = 0.167
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="medical_air_purifier")

    shell_white = model.material("shell_white", rgba=(0.93, 0.95, 0.96, 1.0))
    panel_white = model.material("panel_white", rgba=(0.88, 0.90, 0.92, 1.0))
    charcoal = model.material("charcoal", rgba=(0.20, 0.23, 0.26, 1.0))
    deep_shadow = model.material("deep_shadow", rgba=(0.12, 0.14, 0.16, 1.0))
    latch_dark = model.material("latch_dark", rgba=(0.24, 0.26, 0.29, 1.0))
    filter_media = model.material("filter_media", rgba=(0.61, 0.71, 0.78, 1.0))
    filter_frame_mat = model.material("filter_frame", rgba=(0.72, 0.76, 0.79, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((BODY_W, BODY_H, BACK_T)),
        origin=Origin(xyz=(0.0, 0.0, BACK_T / 2.0)),
        material=shell_white,
        name="back_panel",
    )
    housing.visual(
        Box((SHELL_T, BODY_H, BODY_D - BACK_T)),
        origin=Origin(xyz=(-BODY_W / 2.0 + SHELL_T / 2.0, 0.0, (BODY_D + BACK_T) / 2.0)),
        material=shell_white,
        name="left_shell",
    )
    housing.visual(
        Box((SHELL_T, BODY_H, BODY_D - BACK_T)),
        origin=Origin(xyz=(BODY_W / 2.0 - SHELL_T / 2.0, 0.0, (BODY_D + BACK_T) / 2.0)),
        material=shell_white,
        name="right_shell",
    )
    housing.visual(
        Box((BODY_W - 2.0 * SHELL_T, SHELL_T, BODY_D - BACK_T)),
        origin=Origin(xyz=(0.0, BODY_H / 2.0 - SHELL_T / 2.0, (BODY_D + BACK_T) / 2.0)),
        material=shell_white,
        name="top_shell",
    )
    housing.visual(
        Box((BODY_W - 2.0 * SHELL_T, SHELL_T, BODY_D - BACK_T)),
        origin=Origin(xyz=(0.0, -BODY_H / 2.0 + SHELL_T / 2.0, (BODY_D + BACK_T) / 2.0)),
        material=shell_white,
        name="bottom_shell",
    )

    bezel_height = DOOR_H - 0.02
    housing.visual(
        Box((0.024, bezel_height, FRONT_BEZEL_T)),
        origin=Origin(
            xyz=(-DOOR_W / 2.0 + 0.012, 0.01, BEZEL_Z)
        ),
        material=panel_white,
        name="left_bezel",
    )
    housing.visual(
        Box((0.024, bezel_height, FRONT_BEZEL_T)),
        origin=Origin(
            xyz=(DOOR_W / 2.0 - 0.012, 0.01, BEZEL_Z)
        ),
        material=panel_white,
        name="right_bezel",
    )
    housing.visual(
        Box((DOOR_W, 0.050, FRONT_BEZEL_T)),
        origin=Origin(xyz=(0.0, DOOR_H / 2.0 - 0.005, BEZEL_Z)),
        material=panel_white,
        name="top_bezel",
    )
    housing.visual(
        Box((DOOR_W - 0.04, 0.040, FRONT_BEZEL_T)),
        origin=Origin(xyz=(0.0, HINGE_AXIS_Y + 0.03, BEZEL_Z)),
        material=panel_white,
        name="bottom_bezel",
    )
    housing.visual(
        Box((0.016, 0.60, 0.100)),
        origin=Origin(xyz=(-0.222, 0.02, 0.064)),
        material=filter_frame_mat,
        name="left_filter_guide",
    )
    housing.visual(
        Box((0.016, 0.60, 0.100)),
        origin=Origin(xyz=(0.222, 0.02, 0.064)),
        material=filter_frame_mat,
        name="right_filter_guide",
    )
    housing.visual(
        Box((0.470, 0.610, 0.028)),
        origin=Origin(xyz=(0.0, 0.02, 0.122)),
        material=filter_frame_mat,
        name="filter_frame",
    )
    housing.visual(
        Box((0.438, 0.578, 0.020)),
        origin=Origin(xyz=(0.0, 0.02, 0.126)),
        material=filter_media,
        name="filter_media_core",
    )
    for index in range(9):
        x = -0.176 + index * 0.044
        housing.visual(
            Box((0.014, 0.560, 0.022)),
            origin=Origin(xyz=(x, 0.02, 0.127)),
            material=filter_frame_mat,
            name=f"filter_pleat_{index}",
        )

    housing.inertial = Inertial.from_geometry(
        Box((BODY_W, BODY_H, BODY_D)),
        mass=9.5,
        origin=Origin(xyz=(0.0, 0.0, BODY_D / 2.0)),
    )

    hinge_hardware = model.part("hinge_hardware")
    hinge_hardware.visual(
        Box((0.50, 0.020, 0.008)),
        origin=Origin(xyz=(0.0, 0.024, -0.005)),
        material=panel_white,
        name="mount_strip",
    )
    for index in range(7):
        x = -0.210 + index * HINGE_PITCH
        if index % 2 == 0:
            hinge_hardware.visual(
                Box((HINGE_SEG_LEN, 0.016, 0.010)),
                origin=Origin(xyz=(x, 0.008, -0.003)),
                material=panel_white,
                name=f"frame_tab_{index}",
            )
            hinge_hardware.visual(
                Cylinder(radius=HINGE_RADIUS, length=HINGE_SEG_LEN),
                origin=Origin(
                    xyz=(x, -0.001, 0.006),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=panel_white,
                name=f"frame_knuckle_{index}",
            )
    hinge_hardware.inertial = Inertial.from_geometry(
        Box((0.50, 0.04, 0.03)),
        mass=0.5,
        origin=Origin(xyz=(0.0, 0.009, 0.001)),
    )

    latch_hardware = model.part("latch_hardware")
    latch_hardware.visual(
        Box((0.40, 0.012, 0.006)),
        origin=Origin(xyz=(0.0, 0.001, -0.006)),
        material=panel_white,
        name="crossbar",
    )
    latch_hardware.visual(
        Box((0.022, 0.020, 0.008)),
        origin=Origin(xyz=(-LATCH_X, -0.004, -0.006)),
        material=panel_white,
        name="left_base",
    )
    latch_hardware.visual(
        Box((0.014, 0.010, 0.004)),
        origin=Origin(xyz=(-LATCH_X, -0.012, -0.001)),
        material=latch_dark,
        name="left_striker",
    )
    latch_hardware.visual(
        Box((0.022, 0.020, 0.008)),
        origin=Origin(xyz=(LATCH_X, -0.004, -0.006)),
        material=panel_white,
        name="right_base",
    )
    latch_hardware.visual(
        Box((0.014, 0.010, 0.004)),
        origin=Origin(xyz=(LATCH_X, -0.012, -0.001)),
        material=latch_dark,
        name="right_striker",
    )
    latch_hardware.inertial = Inertial.from_geometry(
        Box((0.44, 0.04, 0.02)),
        mass=0.2,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )

    filter_door = model.part("filter_door")
    filter_door.visual(
        Box((0.488, 0.610, 0.008)),
        origin=Origin(xyz=(0.0, 0.370, -0.006)),
        material=panel_white,
        name="door_skin",
    )
    filter_door.visual(
        Box((0.488, 0.030, 0.012)),
        origin=Origin(xyz=(0.0, 0.060, 0.001)),
        material=panel_white,
        name="bottom_rail",
    )
    filter_door.visual(
        Box((0.400, 0.036, 0.012)),
        origin=Origin(xyz=(0.0, 0.657, 0.001)),
        material=panel_white,
        name="top_rail",
    )
    filter_door.visual(
        Box((0.044, 0.610, 0.012)),
        origin=Origin(xyz=(-0.222, 0.370, 0.001)),
        material=panel_white,
        name="left_stile",
    )
    filter_door.visual(
        Box((0.044, 0.610, 0.012)),
        origin=Origin(xyz=(0.222, 0.370, 0.001)),
        material=panel_white,
        name="right_stile",
    )
    filter_door.visual(
        Box((0.440, 0.018, 0.008)),
        origin=Origin(xyz=(0.0, 0.038, 0.001)),
        material=panel_white,
        name="hinge_strap",
    )
    for index in range(7):
        x = -0.210 + index * HINGE_PITCH
        if index % 2 == 1:
            filter_door.visual(
                Box((HINGE_SEG_LEN, 0.036, 0.010)),
                origin=Origin(xyz=(x, 0.022, -0.003)),
                material=panel_white,
                name=f"door_tab_{index}",
            )
            filter_door.visual(
                Cylinder(radius=HINGE_RADIUS, length=HINGE_SEG_LEN),
                origin=Origin(
                    xyz=(x, 0.0, 0.0),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=panel_white,
                name=f"door_knuckle_{index}",
            )
    filter_door.visual(
        Box((0.430, 0.552, 0.006)),
        origin=Origin(xyz=(0.0, 0.370, -0.0025)),
        material=panel_white,
        name="field_panel",
    )
    filter_door.visual(
        Box((0.228, 0.500, 0.003)),
        origin=Origin(xyz=(0.0, 0.435, -0.001)),
        material=deep_shadow,
        name="grille_recess",
    )
    filter_door.visual(
        Box((0.180, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.640, 0.0005)),
        material=charcoal,
        name="face_grille",
    )
    for index in range(8):
        y = 0.245 + index * 0.055
        filter_door.visual(
            Box((0.180, 0.010, 0.004)),
            origin=Origin(xyz=(0.0, y, 0.0005)),
            material=charcoal,
            name=f"grille_slat_{index}",
        )
    filter_door.visual(
        Box((0.012, 0.440, 0.004)),
        origin=Origin(xyz=(-0.090, 0.435, 0.0005)),
        material=charcoal,
        name="grille_left_rail",
    )
    filter_door.visual(
        Box((0.012, 0.440, 0.004)),
        origin=Origin(xyz=(0.090, 0.435, 0.0005)),
        material=charcoal,
        name="grille_right_rail",
    )
    filter_door.visual(
        Box((0.030, 0.022, 0.010)),
        origin=Origin(xyz=(-LATCH_X, 0.656, 0.001)),
        material=latch_dark,
        name="left_latch",
    )
    filter_door.visual(
        Box((0.016, 0.010, 0.004)),
        origin=Origin(xyz=(-LATCH_X, 0.656, 0.008)),
        material=deep_shadow,
        name="left_latch_button",
    )
    filter_door.visual(
        Box((0.012, 0.016, 0.006)),
        origin=Origin(xyz=(-LATCH_X, 0.656, -0.009)),
        material=latch_dark,
        name="left_hook",
    )
    filter_door.visual(
        Box((0.030, 0.022, 0.010)),
        origin=Origin(xyz=(LATCH_X, 0.656, 0.001)),
        material=latch_dark,
        name="right_latch",
    )
    filter_door.visual(
        Box((0.016, 0.010, 0.004)),
        origin=Origin(xyz=(LATCH_X, 0.656, 0.008)),
        material=deep_shadow,
        name="right_latch_button",
    )
    filter_door.visual(
        Box((0.012, 0.016, 0.006)),
        origin=Origin(xyz=(LATCH_X, 0.656, -0.009)),
        material=latch_dark,
        name="right_hook",
    )
    filter_door.inertial = Inertial.from_geometry(
        Box((0.50, 0.68, 0.03)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.34, 0.0)),
    )

    model.articulation(
        "housing_to_hinge_hardware",
        ArticulationType.FIXED,
        parent=housing,
        child=hinge_hardware,
        origin=Origin(xyz=(0.0, HINGE_AXIS_Y + 0.001, HINGE_AXIS_Z - 0.006)),
    )
    model.articulation(
        "housing_to_latch_hardware",
        ArticulationType.FIXED,
        parent=housing,
        child=latch_hardware,
        origin=Origin(xyz=(0.0, 0.324, 0.159)),
    )

    model.articulation(
        "housing_to_filter_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=filter_door,
        origin=Origin(xyz=(0.0, HINGE_AXIS_Y, HINGE_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=28.0,
            velocity=1.2,
            lower=0.0,
            upper=1.85,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    hinge_hardware = object_model.get_part("hinge_hardware")
    latch_hardware = object_model.get_part("latch_hardware")
    filter_door = object_model.get_part("filter_door")
    door_hinge = object_model.get_articulation("housing_to_filter_door")

    mount_strip = hinge_hardware.get_visual("mount_strip")
    frame_knuckle_2 = hinge_hardware.get_visual("frame_knuckle_2")
    frame_knuckle_4 = hinge_hardware.get_visual("frame_knuckle_4")
    top_bezel = housing.get_visual("top_bezel")
    filter_frame = housing.get_visual("filter_frame")
    left_striker = latch_hardware.get_visual("left_striker")
    right_striker = latch_hardware.get_visual("right_striker")

    field_panel = filter_door.get_visual("field_panel")
    grille_recess = filter_door.get_visual("grille_recess")
    face_grille = filter_door.get_visual("face_grille")
    hinge_strap = filter_door.get_visual("hinge_strap")
    door_knuckle_3 = filter_door.get_visual("door_knuckle_3")
    left_latch = filter_door.get_visual("left_latch")
    left_hook = filter_door.get_visual("left_hook")
    right_latch = filter_door.get_visual("right_latch")
    top_rail = filter_door.get_visual("top_rail")
    right_hook = filter_door.get_visual("right_hook")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_overlap(
        filter_door,
        housing,
        axes="xy",
        min_overlap=0.34,
        elem_a=field_panel,
        elem_b=filter_frame,
        name="door panel spans the filter cavity",
    )
    ctx.expect_overlap(
        filter_door,
        housing,
        axes="xy",
        min_overlap=0.20,
        elem_a=grille_recess,
        elem_b=filter_frame,
        name="narrow face grille stays centered over the filter zone",
    )
    ctx.expect_within(
        filter_door,
        filter_door,
        axes="xy",
        inner_elem=face_grille,
        outer_elem=grille_recess,
        name="the visible grille slats sit inside the recessed vent band",
    )
    ctx.expect_gap(
        filter_door,
        housing,
        axis="z",
        max_gap=0.0015,
        max_penetration=0.0015,
        positive_elem=field_panel,
        negative_elem=top_bezel,
        name="door sits tightly against the housing bezel when shut",
    )
    ctx.expect_contact(
        filter_door,
        latch_hardware,
        elem_a=left_hook,
        elem_b=left_striker,
        name="left detent latch meets its striker in the closed pose",
    )
    ctx.expect_contact(
        filter_door,
        latch_hardware,
        elem_a=right_hook,
        elem_b=right_striker,
        name="right detent latch meets its striker in the closed pose",
    )
    ctx.expect_overlap(
        filter_door,
        filter_door,
        axes="xy",
        min_overlap=0.02,
        elem_a=left_latch,
        elem_b=top_rail,
        name="left latch is mounted in the upper corner band of the door",
    )
    ctx.expect_overlap(
        filter_door,
        filter_door,
        axes="xy",
        min_overlap=0.02,
        elem_a=right_latch,
        elem_b=top_rail,
        name="right latch is mounted in the upper corner band of the door",
    )
    ctx.expect_overlap(
        filter_door,
        hinge_hardware,
        axes="x",
        min_overlap=0.42,
        elem_a=hinge_strap,
        elem_b=mount_strip,
        name="door hinge leaf spans the fixed base mount",
    )
    ctx.expect_gap(
        filter_door,
        hinge_hardware,
        axis="x",
        min_gap=0.004,
        max_gap=0.008,
        positive_elem=door_knuckle_3,
        negative_elem=frame_knuckle_2,
        name="center door knuckle clears the left frame knuckle with hinge-like spacing",
    )
    ctx.expect_gap(
        hinge_hardware,
        filter_door,
        axis="x",
        min_gap=0.004,
        max_gap=0.008,
        positive_elem=frame_knuckle_4,
        negative_elem=door_knuckle_3,
        name="center door knuckle clears the right frame knuckle with hinge-like spacing",
    )

    with ctx.pose({door_hinge: 1.75}):
        ctx.expect_overlap(
            filter_door,
            hinge_hardware,
            axes="x",
            min_overlap=0.42,
            elem_a=hinge_strap,
            elem_b=mount_strip,
            name="hinge mount stays aligned across the swing",
        )
        ctx.expect_gap(
            filter_door,
            housing,
            axis="z",
            min_gap=0.080,
            positive_elem=field_panel,
            negative_elem=top_bezel,
            name="opened door swings forward to expose the filter cavity",
        )
        ctx.expect_gap(
            filter_door,
            latch_hardware,
            axis="z",
            min_gap=0.100,
            positive_elem=left_hook,
            negative_elem=left_striker,
            name="left latch clears the striker when the door is open",
        )
        ctx.expect_gap(
            filter_door,
            latch_hardware,
            axis="z",
            min_gap=0.100,
            positive_elem=right_hook,
            negative_elem=right_striker,
            name="right latch clears the striker when the door is open",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
