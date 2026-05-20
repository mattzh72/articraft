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
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _door_profile_rect(
    *,
    width: float,
    height: float,
    center_y: float = 0.0,
    center_z: float = 0.0,
) -> list[tuple[float, float]]:
    top = center_z + height / 2.0
    bottom = center_z - height / 2.0
    left = center_y - width / 2.0
    right = center_y + width / 2.0
    # The door-shell mesh is authored as an XY profile then rotated around Y so
    # local +Z becomes world +X. After that rotation, world Z is -local X and
    # world Y is local Y.
    return [
        (-top, left),
        (-top, right),
        (-bottom, right),
        (-bottom, left),
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="panel_door_with_pet_flap")

    door_paint = model.material("door_paint", rgba=(0.90, 0.89, 0.84, 1.0))
    door_trim = model.material("door_trim", rgba=(0.83, 0.82, 0.76, 1.0))
    pet_frame = model.material("pet_frame", rgba=(0.19, 0.20, 0.22, 1.0))
    flap_tint = model.material("flap_tint", rgba=(0.50, 0.61, 0.68, 0.42))
    flap_gasket = model.material("flap_gasket", rgba=(0.11, 0.11, 0.12, 1.0))
    latch_brass = model.material("latch_brass", rgba=(0.67, 0.57, 0.33, 1.0))

    door_width = 0.84
    door_height = 2.03
    door_thickness = 0.040

    pet_opening_width = 0.320
    pet_opening_height = 0.420
    pet_opening_lower = -door_height / 2.0 + 0.180
    pet_opening_center_z = pet_opening_lower + pet_opening_height / 2.0
    pet_opening_upper = pet_opening_lower + pet_opening_height

    pet_trim_band = 0.030
    pet_trim_depth = 0.008
    pet_trim_embed = 0.0008

    flap_width = 0.290
    flap_height = 0.390
    flap_thickness = 0.008
    flap_top_z = pet_opening_upper - 0.012

    door_panel = model.part("door_panel")

    door_shell_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _door_profile_rect(width=door_width, height=door_height),
            [
                _door_profile_rect(
                    width=pet_opening_width,
                    height=pet_opening_height,
                    center_z=pet_opening_center_z,
                )
            ],
            door_thickness,
            cap=True,
            center=True,
            closed=True,
        ),
        "door_panel_shell",
    )
    door_panel.visual(
        door_shell_mesh,
        origin=Origin(rpy=(0.0, -math.pi / 2.0, 0.0)),
        material=door_paint,
        name="door_shell",
    )

    upper_panel_outer_width = 0.500
    upper_panel_outer_height = 0.760
    upper_panel_border = 0.060
    upper_panel_center_z = 0.380
    molding_depth = 0.005
    molding_embed = 0.0008
    molding_face_x = door_thickness / 2.0 + molding_depth / 2.0 - molding_embed
    rear_molding_face_x = -molding_face_x

    for prefix, face_x in (
        ("front", molding_face_x),
        ("back", rear_molding_face_x),
    ):
        door_panel.visual(
            Box((molding_depth, upper_panel_outer_width, upper_panel_border)),
            origin=Origin(
                xyz=(
                    face_x,
                    0.0,
                    upper_panel_center_z + upper_panel_outer_height / 2.0 - upper_panel_border / 2.0,
                )
            ),
            material=door_trim,
            name=f"upper_panel_top_{prefix}",
        )
        door_panel.visual(
            Box((molding_depth, upper_panel_outer_width, upper_panel_border)),
            origin=Origin(
                xyz=(
                    face_x,
                    0.0,
                    upper_panel_center_z - upper_panel_outer_height / 2.0 + upper_panel_border / 2.0,
                )
            ),
            material=door_trim,
            name=f"upper_panel_bottom_{prefix}",
        )
        door_panel.visual(
            Box(
                (
                    molding_depth,
                    upper_panel_border,
                    upper_panel_outer_height - 2.0 * upper_panel_border,
                )
            ),
            origin=Origin(
                xyz=(
                    face_x,
                    -upper_panel_outer_width / 2.0 + upper_panel_border / 2.0,
                    upper_panel_center_z,
                )
            ),
            material=door_trim,
            name=f"upper_panel_left_{prefix}",
        )
        door_panel.visual(
            Box(
                (
                    molding_depth,
                    upper_panel_border,
                    upper_panel_outer_height - 2.0 * upper_panel_border,
                )
            ),
            origin=Origin(
                xyz=(
                    face_x,
                    upper_panel_outer_width / 2.0 - upper_panel_border / 2.0,
                    upper_panel_center_z,
                )
            ),
            material=door_trim,
            name=f"upper_panel_right_{prefix}",
        )

    pet_trim_face_x = door_thickness / 2.0 + pet_trim_depth / 2.0 - pet_trim_embed
    pet_trim_back_x = -pet_trim_face_x
    pet_outer_width = pet_opening_width + 2.0 * pet_trim_band
    pet_outer_height = pet_opening_height + 2.0 * pet_trim_band

    for prefix, face_x in (
        ("front", pet_trim_face_x),
        ("back", pet_trim_back_x),
    ):
        door_panel.visual(
            Box((pet_trim_depth, pet_outer_width, pet_trim_band)),
            origin=Origin(
                xyz=(
                    face_x,
                    0.0,
                    pet_opening_center_z + pet_opening_height / 2.0 + pet_trim_band / 2.0,
                )
            ),
            material=pet_frame,
            name=f"pet_frame_top_{prefix}",
        )
        door_panel.visual(
            Box((pet_trim_depth, pet_outer_width, pet_trim_band)),
            origin=Origin(
                xyz=(
                    face_x,
                    0.0,
                    pet_opening_center_z - pet_opening_height / 2.0 - pet_trim_band / 2.0,
                )
            ),
            material=pet_frame,
            name=f"pet_frame_bottom_{prefix}",
        )
        door_panel.visual(
            Box((pet_trim_depth, pet_trim_band, pet_opening_height)),
            origin=Origin(
                xyz=(
                    face_x,
                    -pet_opening_width / 2.0 - pet_trim_band / 2.0,
                    pet_opening_center_z,
                )
            ),
            material=pet_frame,
            name=f"pet_frame_left_{prefix}",
        )
        door_panel.visual(
            Box((pet_trim_depth, pet_trim_band, pet_opening_height)),
            origin=Origin(
                xyz=(
                    face_x,
                    pet_opening_width / 2.0 + pet_trim_band / 2.0,
                    pet_opening_center_z,
                )
            ),
            material=pet_frame,
            name=f"pet_frame_right_{prefix}",
        )

    hinge_lug_width = 0.012
    hinge_lug_height = 0.024
    hinge_lug_thickness = door_thickness + 0.001
    hinge_lug_center_y = pet_opening_width / 2.0 - hinge_lug_width / 2.0 + 0.002
    door_panel.visual(
        Box((hinge_lug_thickness, hinge_lug_width, hinge_lug_height)),
        origin=Origin(xyz=(0.0, -hinge_lug_center_y, flap_top_z - hinge_lug_height / 2.0)),
        material=pet_frame,
        name="pet_hinge_lug_left",
    )
    door_panel.visual(
        Box((hinge_lug_thickness, hinge_lug_width, hinge_lug_height)),
        origin=Origin(xyz=(0.0, hinge_lug_center_y, flap_top_z - hinge_lug_height / 2.0)),
        material=pet_frame,
        name="pet_hinge_lug_right",
    )

    knob_y = door_width / 2.0 - 0.070
    latch_z = -door_height / 2.0 + 1.000
    knob_length = 0.028
    knob_radius = 0.024
    spindle_radius = 0.007
    door_panel.visual(
        Cylinder(radius=knob_radius, length=knob_length),
        origin=Origin(
            xyz=(door_thickness / 2.0 + knob_length / 2.0 - 0.001, knob_y, latch_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=latch_brass,
        name="front_knob",
    )
    door_panel.visual(
        Cylinder(radius=knob_radius, length=knob_length),
        origin=Origin(
            xyz=(-door_thickness / 2.0 - knob_length / 2.0 + 0.001, knob_y, latch_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=latch_brass,
        name="back_knob",
    )
    door_panel.visual(
        Cylinder(radius=spindle_radius, length=door_thickness + 0.006),
        origin=Origin(
            xyz=(0.0, knob_y, latch_z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=latch_brass,
        name="latch_spindle",
    )
    door_panel.inertial = Inertial.from_geometry(
        Box((door_thickness, door_width, door_height)),
        mass=26.0,
        origin=Origin(),
    )

    pet_flap = model.part("pet_flap")
    flap_panel_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(flap_height, flap_width, radius=0.018, corner_segments=8),
            flap_thickness,
            cap=True,
            center=True,
            closed=True,
        ),
        "pet_flap_panel",
    )
    pet_flap.visual(
        flap_panel_mesh,
        origin=Origin(xyz=(0.0, 0.0, -flap_height / 2.0), rpy=(0.0, -math.pi / 2.0, 0.0)),
        material=flap_tint,
        name="flap_panel",
    )
    pet_flap.visual(
        Box((0.014, flap_width + 0.010, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=flap_gasket,
        name="flap_top_cap",
    )
    pet_flap.visual(
        Cylinder(radius=0.007, length=flap_width - 0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=flap_gasket,
        name="flap_hinge_barrel",
    )
    pet_flap.visual(
        Box((0.012, flap_width - 0.040, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -flap_height + 0.014)),
        material=flap_gasket,
        name="flap_bottom_weight",
    )
    pet_flap.inertial = Inertial.from_geometry(
        Box((flap_thickness, flap_width, flap_height)),
        mass=1.2,
        origin=Origin(xyz=(0.0, 0.0, -flap_height / 2.0)),
    )

    model.articulation(
        "door_to_pet_flap",
        ArticulationType.REVOLUTE,
        parent=door_panel,
        child=pet_flap,
        origin=Origin(xyz=(0.0, 0.0, flap_top_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=3.0,
            lower=-1.2,
            upper=1.2,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    door_panel = object_model.get_part("door_panel")
    pet_flap = object_model.get_part("pet_flap")
    flap_hinge = object_model.get_articulation("door_to_pet_flap")

    flap_panel = pet_flap.get_visual("flap_panel")
    flap_top_cap = pet_flap.get_visual("flap_top_cap")
    left_frame = door_panel.get_visual("pet_frame_left_front")
    right_frame = door_panel.get_visual("pet_frame_right_front")
    top_frame = door_panel.get_visual("pet_frame_top_front")
    left_lug = door_panel.get_visual("pet_hinge_lug_left")
    right_lug = door_panel.get_visual("pet_hinge_lug_right")

    limits = flap_hinge.motion_limits
    ctx.check(
        "pet flap uses a horizontal top hinge axis",
        flap_hinge.articulation_type == ArticulationType.REVOLUTE
        and flap_hinge.axis == (0.0, -1.0, 0.0),
        details=f"type={flap_hinge.articulation_type}, axis={flap_hinge.axis}",
    )
    ctx.check(
        "pet flap supports two-way swing travel",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower <= -1.0
        and limits.upper >= 1.0,
        details=f"limits={limits}",
    )

    with ctx.pose({flap_hinge: 0.0}):
        ctx.expect_gap(
            pet_flap,
            door_panel,
            axis="y",
            positive_elem=flap_panel,
            negative_elem=left_frame,
            min_gap=0.010,
            max_gap=0.020,
            name="closed flap clears left frame jamb",
        )
        ctx.expect_gap(
            door_panel,
            pet_flap,
            axis="y",
            positive_elem=right_frame,
            negative_elem=flap_panel,
            min_gap=0.010,
            max_gap=0.020,
            name="closed flap clears right frame jamb",
        )
        ctx.expect_gap(
            door_panel,
            pet_flap,
            axis="z",
            positive_elem=top_frame,
            negative_elem=flap_panel,
            min_gap=0.010,
            max_gap=0.020,
            name="closed flap hangs just below the header hinge rail",
        )
        ctx.expect_contact(
            pet_flap,
            door_panel,
            elem_a=flap_top_cap,
            elem_b=left_lug,
            contact_tol=0.0005,
            name="left hinge lug physically supports the flap",
        )
        ctx.expect_contact(
            pet_flap,
            door_panel,
            elem_a=flap_top_cap,
            elem_b=right_lug,
            contact_tol=0.0005,
            name="right hinge lug physically supports the flap",
        )
        closed_aabb = ctx.part_element_world_aabb(pet_flap, elem="flap_panel")

    with ctx.pose({flap_hinge: 0.9}):
        opened_aabb = ctx.part_element_world_aabb(pet_flap, elem="flap_panel")

    ctx.check(
        "positive flap motion swings the lower edge outward",
        closed_aabb is not None
        and opened_aabb is not None
        and opened_aabb[1][0] > closed_aabb[1][0] + 0.10
        and opened_aabb[0][2] > closed_aabb[0][2] + 0.08,
        details=f"closed={closed_aabb}, opened={opened_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
