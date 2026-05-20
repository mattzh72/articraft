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
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _barrel_body_mesh():
    outer = LatheGeometry(
        [
            (0.0, -0.16),
            (0.066, -0.16),
            (0.074, -0.10),
            (0.075, 0.02),
            (0.069, 0.22),
            (0.061, 0.44),
            (0.056, 0.58),
            (0.053, 0.60),
            (0.0, 0.60),
        ],
        segments=72,
    )
    bore = CylinderGeometry(radius=0.028, height=0.53, radial_segments=56).translate(0.0, 0.0, 0.355)
    return boolean_difference(outer, bore).rotate_y(math.pi / 2.0)


def _muzzle_flare_mesh():
    outer = CylinderGeometry(radius=0.095, height=0.070, radial_segments=56)
    inner = CylinderGeometry(radius=0.030, height=0.078, radial_segments=56)
    return boolean_difference(outer, inner).rotate_y(math.pi / 2.0)


def _swivel_collar_mesh():
    outer = CylinderGeometry(radius=0.112, height=0.10, radial_segments=56)
    inner = CylinderGeometry(radius=0.040, height=0.106, radial_segments=56)
    return boolean_difference(outer, inner)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="naval_swivel_deck_cannon", assets=ASSETS)

    dark_iron = model.material("dark_iron", rgba=(0.16, 0.17, 0.18, 1.0))
    weathered_iron = model.material("weathered_iron", rgba=(0.24, 0.25, 0.27, 1.0))
    blackened_steel = model.material("blackened_steel", rgba=(0.09, 0.10, 0.11, 1.0))

    barrel_mesh = _save_mesh("swivel_cannon_barrel.obj", _barrel_body_mesh())
    muzzle_flare_mesh = _save_mesh("swivel_cannon_muzzle_flare.obj", _muzzle_flare_mesh())
    swivel_collar_mesh = _save_mesh("swivel_cannon_swivel_collar.obj", _swivel_collar_mesh())

    mount = model.part("mount")
    mount.visual(
        Cylinder(radius=0.180, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=weathered_iron,
        name="base_flange",
    )
    for index in range(6):
        angle = index * math.tau / 6.0
        mount.visual(
            Cylinder(radius=0.016, length=0.014),
            origin=Origin(
                xyz=(
                    0.125 * math.cos(angle),
                    0.125 * math.sin(angle),
                    0.047,
                )
            ),
            material=blackened_steel,
            name=f"flange_bolt_{index}",
        )
    mount.visual(
        Cylinder(radius=0.072, length=0.520),
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        material=weathered_iron,
        name="post_column",
    )
    mount.visual(
        Cylinder(radius=0.095, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.585)),
        material=weathered_iron,
        name="post_head",
    )
    mount.visual(
        Cylinder(radius=0.028, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, 0.680)),
        material=blackened_steel,
        name="pivot_pin",
    )
    mount.inertial = Inertial.from_geometry(
        Cylinder(radius=0.180, length=0.770),
        mass=95.0,
        origin=Origin(xyz=(0.0, 0.0, 0.385)),
    )

    yoke = model.part("yoke")
    yoke.visual(
        swivel_collar_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=weathered_iron,
        name="swivel_collar",
    )
    for side_name, side_y in (("left", 0.110), ("right", -0.110)):
        yoke.visual(
            Box((0.160, 0.050, 0.060)),
            origin=Origin(xyz=(0.010, side_y, 0.100)),
            material=weathered_iron,
            name=f"{side_name}_base_shoulder",
        )
        yoke.visual(
            Box((0.060, 0.035, 0.300)),
            origin=Origin(xyz=(-0.046, side_y, 0.240)),
            material=weathered_iron,
            name=f"{side_name}_rear_arm",
        )
        yoke.visual(
            Box((0.060, 0.035, 0.240)),
            origin=Origin(xyz=(0.066, side_y, 0.220)),
            material=weathered_iron,
            name=f"{side_name}_front_arm",
        )
        yoke.visual(
            Box((0.210, 0.035, 0.050)),
            origin=Origin(xyz=(0.010, side_y, 0.365)),
            material=weathered_iron,
            name=f"{side_name}_top_cap",
        )
    yoke.inertial = Inertial.from_geometry(
        Box((0.300, 0.300, 0.420)),
        mass=32.0,
        origin=Origin(xyz=(0.0, 0.0, 0.190)),
    )

    barrel = model.part("barrel")
    barrel.visual(
        barrel_mesh,
        material=dark_iron,
        name="barrel_body",
    )
    barrel.visual(
        muzzle_flare_mesh,
        origin=Origin(xyz=(0.623, 0.0, 0.0)),
        material=dark_iron,
        name="muzzle_flare",
    )
    barrel.visual(
        Cylinder(radius=0.079, length=0.100),
        origin=Origin(xyz=(-0.020, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_iron,
        name="breech_reinforce",
    )
    barrel.visual(
        Cylinder(radius=0.021, length=0.120),
        origin=Origin(xyz=(-0.180, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_iron,
        name="cascabel_neck",
    )
    barrel.visual(
        Sphere(radius=0.039),
        origin=Origin(xyz=(-0.273, 0.0, 0.0)),
        material=dark_iron,
        name="cascabel_knob",
    )
    barrel.visual(
        Cylinder(radius=0.026, length=0.090),
        origin=Origin(xyz=(0.0, 0.113, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=blackened_steel,
        name="left_trunnion",
    )
    barrel.visual(
        Cylinder(radius=0.026, length=0.090),
        origin=Origin(xyz=(0.0, -0.113, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=blackened_steel,
        name="right_trunnion",
    )
    barrel.inertial = Inertial.from_geometry(
        Box((0.980, 0.180, 0.180)),
        mass=52.0,
        origin=Origin(xyz=(0.170, 0.0, 0.0)),
    )

    model.articulation(
        "post_swivel",
        ArticulationType.CONTINUOUS,
        parent=mount,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.610)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.2),
    )
    model.articulation(
        "barrel_elevation",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=barrel,
        origin=Origin(xyz=(0.010, 0.0, 0.270)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.9,
            lower=-0.18,
            upper=0.70,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    mount = object_model.get_part("mount")
    yoke = object_model.get_part("yoke")
    barrel = object_model.get_part("barrel")
    post_swivel = object_model.get_articulation("post_swivel")
    barrel_elevation = object_model.get_articulation("barrel_elevation")

    base_flange = mount.get_visual("base_flange")
    post_column = mount.get_visual("post_column")
    post_head = mount.get_visual("post_head")
    pivot_pin = mount.get_visual("pivot_pin")

    swivel_collar = yoke.get_visual("swivel_collar")
    left_rear_arm = yoke.get_visual("left_rear_arm")
    right_rear_arm = yoke.get_visual("right_rear_arm")
    left_front_arm = yoke.get_visual("left_front_arm")
    right_front_arm = yoke.get_visual("right_front_arm")

    barrel_body = barrel.get_visual("barrel_body")
    muzzle_flare = barrel.get_visual("muzzle_flare")
    cascabel_knob = barrel.get_visual("cascabel_knob")
    left_trunnion = barrel.get_visual("left_trunnion")
    right_trunnion = barrel.get_visual("right_trunnion")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.10)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_within(
        mount,
        mount,
        axes="xy",
        inner_elem=post_column,
        outer_elem=base_flange,
        name="post sits within the broader flanged base plate",
    )
    ctx.expect_gap(
        yoke,
        mount,
        axis="z",
        max_gap=0.002,
        max_penetration=0.001,
        positive_elem=swivel_collar,
        negative_elem=post_head,
        name="swivel collar seats on the post head",
    )
    ctx.expect_within(
        mount,
        yoke,
        axes="xy",
        inner_elem=pivot_pin,
        outer_elem=swivel_collar,
        name="swivel collar wraps concentrically around the vertical post pin",
    )
    ctx.expect_gap(
        barrel,
        yoke,
        axis="z",
        min_gap=0.05,
        positive_elem=barrel_body,
        negative_elem=swivel_collar,
        name="barrel rides above the swivel collar",
    )
    ctx.expect_gap(
        barrel,
        yoke,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=left_trunnion,
        negative_elem=left_rear_arm,
        name="left trunnion bears on the left slot rear wall",
    )
    ctx.expect_gap(
        yoke,
        barrel,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=left_front_arm,
        negative_elem=left_trunnion,
        name="left trunnion bears on the left slot front wall",
    )
    ctx.expect_gap(
        barrel,
        yoke,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=right_trunnion,
        negative_elem=right_rear_arm,
        name="right trunnion bears on the right slot rear wall",
    )
    ctx.expect_gap(
        yoke,
        barrel,
        axis="x",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=right_front_arm,
        negative_elem=right_trunnion,
        name="right trunnion bears on the right slot front wall",
    )
    ctx.expect_gap(
        barrel,
        mount,
        axis="z",
        min_gap=0.14,
        positive_elem=barrel_body,
        negative_elem=post_column,
        name="barrel clears the iron post",
    )
    ctx.expect_within(
        barrel,
        barrel,
        axes="yz",
        inner_elem=barrel_body,
        outer_elem=muzzle_flare,
        name="muzzle flare is visibly wider than the barrel tube",
    )
    ctx.expect_gap(
        barrel,
        yoke,
        axis="x",
        min_gap=0.45,
        positive_elem=muzzle_flare,
        negative_elem=left_front_arm,
        name="flared muzzle projects well ahead of the fork yoke",
    )
    ctx.expect_gap(
        yoke,
        barrel,
        axis="x",
        min_gap=0.10,
        positive_elem=left_rear_arm,
        negative_elem=cascabel_knob,
        name="cascabel knob sits aft of the yoke arms",
    )
    ctx.expect_within(
        barrel,
        barrel,
        axes="yz",
        inner_elem=barrel.get_visual("cascabel_neck"),
        outer_elem=cascabel_knob,
        name="cascabel knob reads larger than its breech neck",
    )

    with ctx.pose({barrel_elevation: 0.60}):
        ctx.expect_gap(
            barrel,
            mount,
            axis="z",
            min_gap=0.34,
            positive_elem=muzzle_flare,
            negative_elem=post_head,
            name="elevated muzzle rises clearly above the post head",
        )
        ctx.expect_gap(
            barrel,
            yoke,
            axis="x",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=left_trunnion,
            negative_elem=left_rear_arm,
            name="left trunnion remains seated against the rear wall when elevated",
        )

    with ctx.pose({post_swivel: math.pi / 2.0}):
        ctx.expect_gap(
            yoke,
            mount,
            axis="z",
            max_gap=0.002,
            max_penetration=0.001,
            positive_elem=swivel_collar,
            negative_elem=post_head,
            name="swivel collar stays seated while turned broadside",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
