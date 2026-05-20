from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


OPENING_WIDTH = 0.52
OPENING_HEIGHT = 1.06
SURROUND_JAMB = 0.07
SURROUND_HEAD = 0.08
SURROUND_DEPTH = 0.07

PANEL_WIDTH = 0.50
PANEL_DEPTH = 0.028
PANEL_STILE = 0.045
PANEL_RAIL = 0.045

UPPER_PANEL_HEIGHT = 0.28
LOWER_PANEL_HEIGHT = 0.34
UPPER_PANEL_Z = 0.23
LOWER_PANEL_Z = -0.21

UPPER_LOUVER_COUNT = 3
LOWER_LOUVER_COUNT = 4

HINGE_X = -PANEL_WIDTH / 2.0
PANEL_CENTER_Y = SURROUND_DEPTH / 2.0 + PANEL_DEPTH / 2.0


def _add_louvered_panel(
    model: ArticulatedObject,
    surround_name: str,
    *,
    panel_name: str,
    hinge_name: str,
    z_center: float,
    panel_height: float,
    louver_count: int,
    panel_material: str,
    louver_material: str,
    hardware_material: str,
) -> None:
    panel = model.part(panel_name)

    panel.visual(
        Box((PANEL_STILE, PANEL_DEPTH, panel_height)),
        origin=Origin(xyz=(PANEL_STILE / 2.0, 0.0, 0.0)),
        material=panel_material,
        name=f"{panel_name}_left_stile",
    )
    panel.visual(
        Box((PANEL_STILE, PANEL_DEPTH, panel_height)),
        origin=Origin(xyz=(PANEL_WIDTH - PANEL_STILE / 2.0, 0.0, 0.0)),
        material=panel_material,
        name=f"{panel_name}_right_stile",
    )
    panel.visual(
        Box((PANEL_WIDTH, PANEL_DEPTH, PANEL_RAIL)),
        origin=Origin(xyz=(PANEL_WIDTH / 2.0, 0.0, panel_height / 2.0 - PANEL_RAIL / 2.0)),
        material=panel_material,
        name=f"{panel_name}_top_rail",
    )
    panel.visual(
        Box((PANEL_WIDTH, PANEL_DEPTH, PANEL_RAIL)),
        origin=Origin(xyz=(PANEL_WIDTH / 2.0, 0.0, -panel_height / 2.0 + PANEL_RAIL / 2.0)),
        material=panel_material,
        name=f"{panel_name}_bottom_rail",
    )

    hinge_barrel_radius = 0.007
    hinge_barrel_length = 0.055
    for index, barrel_z in enumerate((-panel_height * 0.28, panel_height * 0.28), start=1):
        panel.visual(
            Cylinder(radius=hinge_barrel_radius, length=hinge_barrel_length),
            origin=Origin(xyz=(0.0, 0.0, barrel_z)),
            material=hardware_material,
            name=f"{panel_name}_hinge_barrel_{index}",
        )

    model.articulation(
        hinge_name,
        ArticulationType.REVOLUTE,
        parent=surround_name,
        child=panel,
        origin=Origin(xyz=(HINGE_X, PANEL_CENTER_Y, z_center)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=0.0,
            upper=1.45,
        ),
    )

    clear_width = PANEL_WIDTH - 2.0 * PANEL_STILE
    clear_height = panel_height - 2.0 * PANEL_RAIL
    pitch = clear_height / (louver_count + 1)
    blade_depth = 0.012
    blade_height = min(0.038, pitch * 0.72)
    blade_length = clear_width - 0.008
    pin_length = 0.008

    for index in range(1, louver_count + 1):
        louver = model.part(f"{panel_name}_louver_{index}")
        louver.visual(
            Box((blade_length, blade_depth, blade_height)),
            origin=Origin(rpy=(0.24, 0.0, 0.0)),
            material=louver_material,
            name="blade",
        )
        louver.visual(
            Cylinder(radius=0.004, length=pin_length),
            origin=Origin(xyz=(-blade_length / 2.0, 0.0, 0.0), rpy=(0.0, 1.57079632679, 0.0)),
            material=hardware_material,
            name="left_pivot",
        )
        louver.visual(
            Cylinder(radius=0.004, length=pin_length),
            origin=Origin(xyz=(blade_length / 2.0, 0.0, 0.0), rpy=(0.0, 1.57079632679, 0.0)),
            material=hardware_material,
            name="right_pivot",
        )

        louver_z = clear_height / 2.0 - pitch * index
        model.articulation(
            f"{panel_name}_louver_{index}_pivot",
            ArticulationType.REVOLUTE,
            parent=panel,
            child=louver,
            origin=Origin(xyz=(PANEL_WIDTH / 2.0, 0.0, louver_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=3.0,
                lower=-0.35,
                upper=0.55,
            ),
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cafe_shutter")

    trim = model.material("trim_white", color=(0.94, 0.92, 0.88))
    shutter_paint = model.material("shutter_green", color=(0.18, 0.30, 0.23))
    louver_paint = model.material("louver_green", color=(0.22, 0.36, 0.27))
    hinge_metal = model.material("hinge_bronze", color=(0.30, 0.24, 0.18))
    recess_shadow = model.material("recess_shadow", color=(0.20, 0.18, 0.16))

    surround = model.part("window_surround")
    outer_width = OPENING_WIDTH + 2.0 * SURROUND_JAMB
    outer_height = OPENING_HEIGHT + 2.0 * SURROUND_HEAD

    surround.visual(
        Box((SURROUND_JAMB, SURROUND_DEPTH, outer_height)),
        origin=Origin(xyz=(-OPENING_WIDTH / 2.0 - SURROUND_JAMB / 2.0, 0.0, 0.0)),
        material=trim,
        name="left_jamb",
    )
    surround.visual(
        Box((SURROUND_JAMB, SURROUND_DEPTH, outer_height)),
        origin=Origin(xyz=(OPENING_WIDTH / 2.0 + SURROUND_JAMB / 2.0, 0.0, 0.0)),
        material=trim,
        name="right_jamb",
    )
    surround.visual(
        Box((OPENING_WIDTH, SURROUND_DEPTH, SURROUND_HEAD)),
        origin=Origin(xyz=(0.0, 0.0, OPENING_HEIGHT / 2.0 + SURROUND_HEAD / 2.0)),
        material=trim,
        name="head_casing",
    )
    surround.visual(
        Box((OPENING_WIDTH, SURROUND_DEPTH, SURROUND_HEAD)),
        origin=Origin(xyz=(0.0, 0.0, -OPENING_HEIGHT / 2.0 - SURROUND_HEAD / 2.0)),
        material=trim,
        name="sill_casing",
    )

    reveal_depth = 0.032
    reveal_thickness = 0.018
    reveal_y = -SURROUND_DEPTH / 2.0 + reveal_depth / 2.0
    surround.visual(
        Box((reveal_thickness, reveal_depth, OPENING_HEIGHT)),
        origin=Origin(xyz=(-OPENING_WIDTH / 2.0 + reveal_thickness / 2.0, reveal_y, 0.0)),
        material=recess_shadow,
        name="left_reveal",
    )
    surround.visual(
        Box((reveal_thickness, reveal_depth, OPENING_HEIGHT)),
        origin=Origin(xyz=(OPENING_WIDTH / 2.0 - reveal_thickness / 2.0, reveal_y, 0.0)),
        material=recess_shadow,
        name="right_reveal",
    )
    surround.visual(
        Box((OPENING_WIDTH, reveal_depth, reveal_thickness)),
        origin=Origin(xyz=(0.0, reveal_y, OPENING_HEIGHT / 2.0 - reveal_thickness / 2.0)),
        material=recess_shadow,
        name="top_reveal",
    )
    surround.visual(
        Box((OPENING_WIDTH, reveal_depth, reveal_thickness)),
        origin=Origin(xyz=(0.0, reveal_y, -OPENING_HEIGHT / 2.0 + reveal_thickness / 2.0)),
        material=recess_shadow,
        name="bottom_reveal",
    )

    mount_strip_width = 0.038
    mount_strip_depth = 0.012
    mount_strip_y = SURROUND_DEPTH / 2.0 - mount_strip_depth / 2.0
    surround.visual(
        Box((mount_strip_width, mount_strip_depth, UPPER_PANEL_HEIGHT * 0.72)),
        origin=Origin(
            xyz=(-OPENING_WIDTH / 2.0 + mount_strip_width / 2.0, mount_strip_y, UPPER_PANEL_Z)
        ),
        material=hinge_metal,
        name="upper_mount_strip",
    )
    surround.visual(
        Box((mount_strip_width, mount_strip_depth, LOWER_PANEL_HEIGHT * 0.72)),
        origin=Origin(
            xyz=(-OPENING_WIDTH / 2.0 + mount_strip_width / 2.0, mount_strip_y, LOWER_PANEL_Z)
        ),
        material=hinge_metal,
        name="lower_mount_strip",
    )

    _add_louvered_panel(
        model,
        surround.name,
        panel_name="upper_panel",
        hinge_name="upper_panel_hinge",
        z_center=UPPER_PANEL_Z,
        panel_height=UPPER_PANEL_HEIGHT,
        louver_count=UPPER_LOUVER_COUNT,
        panel_material=shutter_paint,
        louver_material=louver_paint,
        hardware_material=hinge_metal,
    )
    _add_louvered_panel(
        model,
        surround.name,
        panel_name="lower_panel",
        hinge_name="lower_panel_hinge",
        z_center=LOWER_PANEL_Z,
        panel_height=LOWER_PANEL_HEIGHT,
        louver_count=LOWER_LOUVER_COUNT,
        panel_material=shutter_paint,
        louver_material=louver_paint,
        hardware_material=hinge_metal,
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

    surround = object_model.get_part("window_surround")
    upper_panel = object_model.get_part("upper_panel")
    lower_panel = object_model.get_part("lower_panel")
    upper_hinge = object_model.get_articulation("upper_panel_hinge")
    lower_hinge = object_model.get_articulation("lower_panel_hinge")

    ctx.check(
        "upper panel uses a vertical hinge axis",
        tuple(upper_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"axis={upper_hinge.axis}",
    )
    ctx.check(
        "lower panel uses a vertical hinge axis",
        tuple(lower_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"axis={lower_hinge.axis}",
    )

    upper_louver_axes_ok = True
    lower_louver_axes_ok = True
    for index in range(1, UPPER_LOUVER_COUNT + 1):
        axis = object_model.get_articulation(f"upper_panel_louver_{index}_pivot").axis
        upper_louver_axes_ok = upper_louver_axes_ok and tuple(axis) == (1.0, 0.0, 0.0)
    for index in range(1, LOWER_LOUVER_COUNT + 1):
        axis = object_model.get_articulation(f"lower_panel_louver_{index}_pivot").axis
        lower_louver_axes_ok = lower_louver_axes_ok and tuple(axis) == (1.0, 0.0, 0.0)

    ctx.check(
        "upper panel louvers pivot on their long axis",
        upper_louver_axes_ok,
        details="One or more upper louvers does not use the x-axis pivot.",
    )
    ctx.check(
        "lower panel louvers pivot on their long axis",
        lower_louver_axes_ok,
        details="One or more lower louvers does not use the x-axis pivot.",
    )

    with ctx.pose({upper_hinge: 0.0, lower_hinge: 0.0}):
        ctx.expect_contact(
            upper_panel,
            surround,
            name="upper panel is mounted to the surround",
        )
        ctx.expect_contact(
            lower_panel,
            surround,
            name="lower panel is mounted to the surround",
        )
        ctx.expect_gap(
            upper_panel,
            lower_panel,
            axis="z",
            min_gap=0.10,
            max_gap=0.14,
            name="upper and lower panels remain independently framed",
        )

        for index in range(1, UPPER_LOUVER_COUNT + 1):
            ctx.expect_contact(
                object_model.get_part(f"upper_panel_louver_{index}"),
                upper_panel,
                name=f"upper louver {index} is carried by the upper panel stiles",
            )
        for index in range(1, LOWER_LOUVER_COUNT + 1):
            ctx.expect_contact(
                object_model.get_part(f"lower_panel_louver_{index}"),
                lower_panel,
                name=f"lower louver {index} is carried by the lower panel stiles",
            )

    closed_upper_right = ctx.part_element_world_aabb(upper_panel, elem="upper_panel_right_stile")
    with ctx.pose({upper_hinge: 0.95}):
        opened_upper_right = ctx.part_element_world_aabb(upper_panel, elem="upper_panel_right_stile")
    ctx.check(
        "upper panel opens outward from the surround",
        closed_upper_right is not None
        and opened_upper_right is not None
        and opened_upper_right[1][1] > closed_upper_right[1][1] + 0.14,
        details=f"closed={closed_upper_right}, opened={opened_upper_right}",
    )

    closed_lower_right = ctx.part_element_world_aabb(lower_panel, elem="lower_panel_right_stile")
    with ctx.pose({lower_hinge: 0.95}):
        opened_lower_right = ctx.part_element_world_aabb(lower_panel, elem="lower_panel_right_stile")
    ctx.check(
        "lower panel opens outward from the surround",
        closed_lower_right is not None
        and opened_lower_right is not None
        and opened_lower_right[1][1] > closed_lower_right[1][1] + 0.14,
        details=f"closed={closed_lower_right}, opened={opened_lower_right}",
    )

    upper_mid_louver = object_model.get_part("upper_panel_louver_2")
    closed_upper_blade = ctx.part_element_world_aabb(upper_mid_louver, elem="blade")
    with ctx.pose({"upper_panel_louver_2_pivot": 0.45}):
        pitched_upper_blade = ctx.part_element_world_aabb(upper_mid_louver, elem="blade")
    ctx.check(
        "upper louvers visibly change pitch",
        closed_upper_blade is not None
        and pitched_upper_blade is not None
        and (pitched_upper_blade[1][1] - pitched_upper_blade[0][1])
        > (closed_upper_blade[1][1] - closed_upper_blade[0][1]) + 0.01,
        details=f"closed={closed_upper_blade}, pitched={pitched_upper_blade}",
    )

    lower_mid_louver = object_model.get_part("lower_panel_louver_2")
    closed_lower_blade = ctx.part_element_world_aabb(lower_mid_louver, elem="blade")
    with ctx.pose({"lower_panel_louver_2_pivot": 0.45}):
        pitched_lower_blade = ctx.part_element_world_aabb(lower_mid_louver, elem="blade")
    ctx.check(
        "lower louvers visibly change pitch",
        closed_lower_blade is not None
        and pitched_lower_blade is not None
        and (pitched_lower_blade[1][1] - pitched_lower_blade[0][1])
        > (closed_lower_blade[1][1] - closed_lower_blade[0][1]) + 0.01,
        details=f"closed={closed_lower_blade}, pitched={pitched_lower_blade}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
