from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    rounded_rect_profile,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _section_from_profile(
    profile: list[tuple[float, float]],
    *,
    y: float,
    z_offset: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x, y, z + z_offset) for x, z in profile]


def _barrel_body_mesh():
    sections = []
    for y, width, height, radius, z_offset in (
        (-0.047, 0.034, 0.036, 0.006, -0.004),
        (-0.012, 0.041, 0.044, 0.007, -0.002),
        (0.026, 0.048, 0.052, 0.008, 0.000),
        (0.054, 0.050, 0.050, 0.008, -0.001),
    ):
        sections.append(
            _section_from_profile(
                rounded_rect_profile(width, height, radius, corner_segments=6),
                y=y,
                z_offset=z_offset,
            )
        )
    return repair_loft(section_loft(sections))


def _roof_prism_hump_mesh():
    sections = []
    for y, width, height, z_offset in (
        (-0.026, 0.022, 0.015, 0.014),
        (-0.004, 0.028, 0.020, 0.017),
        (0.022, 0.026, 0.017, 0.015),
    ):
        half_width = width * 0.5
        half_height = height * 0.5
        profile = [
            (-half_width, -half_height),
            (half_width, -half_height),
            (half_width, -0.12 * height),
            (0.0, half_height),
            (-half_width, -0.12 * height),
        ]
        sections.append(_section_from_profile(profile, y=y, z_offset=z_offset))
    return repair_loft(section_loft(sections))


def _objective_shell_mesh():
    return (
        LatheGeometry.from_shell_profiles(
            [
                (0.019, -0.028),
                (0.022, -0.018),
                (0.025, 0.004),
                (0.026, 0.020),
                (0.026, 0.032),
            ],
            [
                (0.015, -0.024),
                (0.017, -0.015),
                (0.020, 0.006),
                (0.021, 0.021),
                (0.021, 0.032),
            ],
            segments=60,
            start_cap="flat",
            end_cap="round",
            lip_samples=8,
        ).rotate_x(-math.pi / 2.0)
    )


def _focus_knob_mesh():
    return (
        LatheGeometry(
            [
                (0.0, -0.015),
                (0.010, -0.015),
                (0.016, -0.012),
                (0.018, -0.008),
                (0.018, 0.008),
                (0.016, 0.012),
                (0.010, 0.015),
                (0.0, 0.015),
            ],
            segments=56,
        ).rotate_y(math.pi / 2.0)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roof_prism_binocular", assets=ASSETS)

    armor = model.material("armor_olive", rgba=(0.23, 0.28, 0.20, 1.0))
    bridge_metal = model.material("bridge_metal", rgba=(0.18, 0.19, 0.20, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    glass = model.material("glass_blue", rgba=(0.45, 0.60, 0.72, 0.42))

    barrel_body = _save_mesh("binocular_barrel_body.obj", _barrel_body_mesh())
    prism_hump = _save_mesh("binocular_prism_hump.obj", _roof_prism_hump_mesh())
    objective_shell = _save_mesh("binocular_objective_shell.obj", _objective_shell_mesh())
    focus_knob_mesh = _save_mesh("binocular_focus_knob.obj", _focus_knob_mesh())

    bridge = model.part("bridge")
    bridge.visual(
        Cylinder(radius=0.0072, length=0.052),
        material=bridge_metal,
        name="hinge_post",
    )
    bridge.visual(
        Cylinder(radius=0.0095, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=bridge_metal,
        name="lower_hinge_collar",
    )
    bridge.visual(
        Box((0.018, 0.030, 0.016)),
        origin=Origin(xyz=(0.0, 0.010, 0.006)),
        material=bridge_metal,
        name="front_bridge_block",
    )
    bridge.visual(
        Box((0.016, 0.026, 0.014)),
        origin=Origin(xyz=(0.0, -0.018, -0.004)),
        material=bridge_metal,
        name="rear_bridge_block",
    )
    bridge.visual(
        Box((0.028, 0.022, 0.010)),
        origin=Origin(xyz=(0.0, -0.003, 0.021)),
        material=bridge_metal,
        name="knob_bridge",
    )
    bridge.visual(
        Box((0.008, 0.018, 0.022)),
        origin=Origin(xyz=(-0.018, -0.003, 0.037)),
        material=bridge_metal,
        name="left_knob_yoke",
    )
    bridge.visual(
        Box((0.008, 0.018, 0.022)),
        origin=Origin(xyz=(0.018, -0.003, 0.037)),
        material=bridge_metal,
        name="right_knob_yoke",
    )
    bridge.inertial = Inertial.from_geometry(
        Box((0.050, 0.056, 0.072)),
        mass=0.30,
        origin=Origin(xyz=(0.0, -0.002, 0.004)),
    )

    def _add_barrel(name: str, *, side_sign: float, sleeve_center_z: float):
        barrel = model.part(name)
        barrel_center_x = side_sign * 0.037

        barrel.visual(
            Cylinder(radius=0.010, length=0.020),
            origin=Origin(xyz=(0.0, 0.0, sleeve_center_z)),
            material=bridge_metal,
            name="hinge_sleeve",
        )
        barrel.visual(
            Box((0.030, 0.018, 0.012)),
            origin=Origin(xyz=(side_sign * 0.016, -0.006, sleeve_center_z * 0.35)),
            material=armor,
            name="bridge_arm",
        )
        barrel.visual(
            Box((0.012, 0.028, 0.020)),
            origin=Origin(xyz=(side_sign * 0.024, 0.004, sleeve_center_z * 0.12)),
            material=armor,
            name="bridge_cheek",
        )
        barrel.visual(
            barrel_body,
            origin=Origin(xyz=(barrel_center_x, 0.0, 0.0)),
            material=armor,
            name="body_shell",
        )
        barrel.visual(
            prism_hump,
            origin=Origin(xyz=(barrel_center_x, 0.0, 0.0)),
            material=armor,
            name="prism_hump",
        )
        barrel.visual(
            objective_shell,
            origin=Origin(xyz=(barrel_center_x, 0.052, 0.0)),
            material=armor,
            name="objective_shell",
        )
        barrel.visual(
            Cylinder(radius=0.0215, length=0.002),
            origin=Origin(
                xyz=(barrel_center_x, 0.074, 0.0),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=glass,
            name="objective_glass",
        )
        barrel.visual(
            Cylinder(radius=0.017, length=0.030),
            origin=Origin(
                xyz=(barrel_center_x, -0.050, 0.0),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=bridge_metal,
            name="ocular_housing",
        )
        barrel.visual(
            Cylinder(radius=0.018, length=0.026),
            origin=Origin(
                xyz=(barrel_center_x, -0.068, 0.0),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=black_rubber,
            name="eyecup_outer",
        )
        barrel.visual(
            Cylinder(radius=0.012, length=0.018),
            origin=Origin(
                xyz=(barrel_center_x, -0.071, 0.0),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=bridge_metal,
            name="eyecup_inner",
        )
        barrel.visual(
            Cylinder(radius=0.011, length=0.002),
            origin=Origin(
                xyz=(barrel_center_x, -0.058, 0.0),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=glass,
            name="ocular_glass",
        )
        barrel.inertial = Inertial.from_geometry(
            Box((0.068, 0.156, 0.060)),
            mass=0.42,
            origin=Origin(xyz=(barrel_center_x, 0.004, 0.0)),
        )
        return barrel

    left_barrel = _add_barrel("left_barrel", side_sign=-1.0, sleeve_center_z=0.011)
    right_barrel = _add_barrel("right_barrel", side_sign=1.0, sleeve_center_z=-0.011)

    focus_knob = model.part("focus_knob")
    focus_knob.visual(focus_knob_mesh, material=black_rubber, name="focus_wheel")
    focus_knob.visual(
        Cylinder(radius=0.005, length=0.040),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bridge_metal,
        name="focus_axle",
    )
    focus_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.018, length=0.030),
        mass=0.08,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "left_hinge",
        ArticulationType.REVOLUTE,
        parent=bridge,
        child=left_barrel,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.5,
            lower=-0.12,
            upper=0.16,
        ),
    )
    model.articulation(
        "right_hinge",
        ArticulationType.REVOLUTE,
        parent=bridge,
        child=right_barrel,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.5,
            lower=-0.16,
            upper=0.12,
        ),
    )
    model.articulation(
        "focus_wheel_joint",
        ArticulationType.REVOLUTE,
        parent=bridge,
        child=focus_knob,
        origin=Origin(xyz=(0.0, -0.003, 0.043)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=6.0,
            lower=-10.0,
            upper=10.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    bridge = object_model.get_part("bridge")
    left_barrel = object_model.get_part("left_barrel")
    right_barrel = object_model.get_part("right_barrel")
    focus_knob = object_model.get_part("focus_knob")
    left_hinge = object_model.get_articulation("left_hinge")
    right_hinge = object_model.get_articulation("right_hinge")
    focus_wheel_joint = object_model.get_articulation("focus_wheel_joint")

    hinge_post = bridge.get_visual("hinge_post")
    knob_bridge = bridge.get_visual("knob_bridge")
    left_sleeve = left_barrel.get_visual("hinge_sleeve")
    right_sleeve = right_barrel.get_visual("hinge_sleeve")
    left_objective = left_barrel.get_visual("objective_glass")
    right_objective = right_barrel.get_visual("objective_glass")
    focus_wheel = focus_knob.get_visual("focus_wheel")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    ctx.allow_overlap(bridge, left_barrel, reason="left hinge sleeve nests around the center hinge post")
    ctx.allow_overlap(bridge, right_barrel, reason="right hinge sleeve nests around the center hinge post")
    ctx.allow_overlap(bridge, focus_knob, reason="focus wheel axle nests inside the bridge yokes")

    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_overlap(
        left_barrel,
        bridge,
        axes="xy",
        min_overlap=0.010,
        elem_a=left_sleeve,
        elem_b=hinge_post,
        name="left barrel sleeve wraps the hinge post",
    )
    ctx.expect_overlap(
        right_barrel,
        bridge,
        axes="xy",
        min_overlap=0.010,
        elem_a=right_sleeve,
        elem_b=hinge_post,
        name="right barrel sleeve wraps the hinge post",
    )
    ctx.expect_gap(
        focus_knob,
        bridge,
        axis="z",
        max_gap=0.001,
        max_penetration=0.002,
        positive_elem=focus_wheel,
        negative_elem=knob_bridge,
        name="focus wheel sits down on the bridge saddle",
    )
    ctx.expect_overlap(
        focus_knob,
        bridge,
        axes="xy",
        min_overlap=0.010,
        elem_a=focus_wheel,
        elem_b=knob_bridge,
        name="focus wheel stays centered over the bridge",
    )
    ctx.expect_gap(
        right_barrel,
        left_barrel,
        axis="x",
        min_gap=0.012,
        positive_elem=right_objective,
        negative_elem=left_objective,
        name="objective lenses remain as two distinct front apertures",
    )

    with ctx.pose({left_hinge: 0.12, right_hinge: -0.12}):
        ctx.expect_gap(
            right_barrel,
            left_barrel,
            axis="x",
            min_gap=0.020,
            positive_elem=right_objective,
            negative_elem=left_objective,
            name="barrels open wider around the center hinge",
        )
        ctx.expect_overlap(
            left_barrel,
            bridge,
            axes="xy",
            min_overlap=0.010,
            elem_a=left_sleeve,
            elem_b=hinge_post,
            name="left sleeve stays seated when opened",
        )
        ctx.expect_overlap(
            right_barrel,
            bridge,
            axes="xy",
            min_overlap=0.010,
            elem_a=right_sleeve,
            elem_b=hinge_post,
            name="right sleeve stays seated when opened",
        )

    with ctx.pose({left_hinge: -0.10, right_hinge: 0.10}):
        ctx.expect_gap(
            right_barrel,
            left_barrel,
            axis="x",
            min_gap=0.008,
            positive_elem=right_objective,
            negative_elem=left_objective,
            name="barrels can close for narrower interpupillary spacing",
        )

    with ctx.pose({focus_wheel_joint: 3.5}):
        ctx.expect_gap(
            focus_knob,
            bridge,
            axis="z",
            max_gap=0.001,
            max_penetration=0.002,
            positive_elem=focus_wheel,
            negative_elem=knob_bridge,
            name="focus wheel stays seated while rotated",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
