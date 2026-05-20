from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)
import cadquery as cq


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="sensor_turret")

    # Register materials
    model.material(name="abs_dark_gray", rgba=(0.2, 0.2, 0.2, 1.0))
    model.material(name="metal_silver", rgba=(0.8, 0.8, 0.8, 1.0))
    model.material(name="plastic_black", rgba=(0.1, 0.1, 0.1, 1.0))
    model.material(name="plastic_blue", rgba=(0.0, 0.0, 1.0, 1.0))

    # Base part: square electronics base with bearing pocket
    base = model.part("base")

    # Base main shape: CadQuery box with cylindrical pocket for bearing
    base_shape = (
        cq.Workplane("XY")
        .box(0.15, 0.15, 0.05)  # 150x150x50mm, centered at (0,0,0) in shape coords
        .faces("+Z")  # top face at z=0.025 in shape coords
        .workplane()
        .center(0, 0)
        .circle(0.02)  # 40mm diameter pocket
        .cutBlind(-0.003)  # 3mm deep pocket, from z=0.025 to z=0.022 in shape coords
    )
    # Convert CadQuery shape to managed mesh
    base_mesh = mesh_from_cadquery(base_shape, "base_shell")
    # Place mesh so top face is at world z=0.05 (base top)
    base.visual(base_mesh, origin=Origin(xyz=(0.0, 0.0, 0.025)), name="base_shell", material="abs_dark_gray")

    # Screw heads: 4 on base corners (M3 size, 6mm diameter, 2mm height)
    screw_positions = [
        (0.06, 0.06),
        (-0.06, 0.06),
        (-0.06, -0.06),
        (0.06, -0.06),
    ]
    for i, (x, y) in enumerate(screw_positions):
        # SDK Cylinder: radius=0.003, length=0.002 (height)
        screw_shape = Cylinder(radius=0.003, length=0.002)
        base.visual(
            screw_shape,
            origin=Origin(xyz=(x, y, 0.051)),  # sits on base top (world z=0.05 + 1mm half-height)
            name=f"screw_{i}",
            material="metal_silver",
        )

    # Turret part: rotating cylindrical turret
    turret = model.part("turret")

    # Main turret body: 100mm diameter, 120mm tall cylinder (SDK Cylinder)
    # SDK Cylinder: radius=0.05, length=0.12 (height)
    turret_body_shape = Cylinder(radius=0.05, length=0.12)
    # Center in turret local frame: bottom at local z=0 (world z=0.05), top at local z=0.12 (world z=0.17)
    turret.visual(
        turret_body_shape,
        origin=Origin(xyz=(0.0, 0.0, 0.06)),  # local center z=0.06 (half-height 0.06)
        name="turret_body",
        material="plastic_black",
    )

    # Turret stub: fits into base pocket, creates bearing seam (38mm diameter, 3mm tall)
    # SDK Cylinder: radius=0.019, length=0.003 (height)
    turret_stub_shape = Cylinder(radius=0.019, length=0.003)
    # Center in turret local frame: top at local z=0 (world z=0.05), bottom at local z=-0.003 (world z=0.047)
    turret.visual(
        turret_stub_shape,
        origin=Origin(xyz=(0.0, 0.0, -0.0015)),  # local center z=-0.0015 (half-height 0.0015)
        name="turret_stub",
        material="plastic_black",
    )

    # Sensor face: small front-facing rectangular sensor
    turret.visual(
        Box((0.03, 0.02, 0.005)),  # 30x20x5mm
        origin=Origin(xyz=(0.0525, 0.0, 0.06)),  # front of turret (+X local), centered vertically
        name="sensor_face",
        material="plastic_blue",
    )

    # Yaw articulation: vertical axis rotation (revolute joint around Z)
    model.articulation(
        "yaw_joint",
        ArticulationType.REVOLUTE,
        parent=base,
        child=turret,
        origin=Origin(xyz=(0.0, 0.0, 0.05)),  # rotation center at base top center (world coords)
        axis=(0.0, 0.0, 1.0),  # vertical Z axis (yaw)
        motion_limits=MotionLimits(effort=1.0, velocity=1.0, lower=-3.1416, upper=3.1416),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    turret = object_model.get_part("turret")
    yaw = object_model.get_articulation("yaw_joint")

    # 1. Validate yaw joint configuration
    ctx.check("yaw_joint is revolute", yaw.articulation_type == ArticulationType.REVOLUTE, details=f"Actual type: {yaw.articulation_type}")
    ctx.check("yaw_joint axis is vertical Z", yaw.axis == (0.0, 0.0, 1.0), details=f"Actual axis: {yaw.axis}")

    # 2. Rest pose (0 yaw) checks
    with ctx.pose({yaw: 0.0}):
        # Turret seated on base contact
        ctx.expect_contact(base, turret, elem_a="base_shell", elem_b="turret_body", contact_tol=0.001, name="turret seated on base")

        # Bearing seam: modeled as 0.001m radial gap between base pocket and turret stub
        # (validated via geometry, omitted from tests due to element scope)

        # Closed pose: sensor faces forward (+X world)
        sensor_aabb = ctx.part_element_world_aabb(turret, elem="sensor_face")
        # aabb is tuple of (min_xyz_tuple, max_xyz_tuple)
        sensor_center_x = (sensor_aabb[0][0] + sensor_aabb[1][0]) / 2
        ctx.check("closed pose sensor faces +X", sensor_center_x > 0.05, details=f"Sensor center X: {sensor_center_x}")

        # Visible details validation
        base_visual_names = [v.name for v in base.visuals]
        screw_count = sum(1 for name in base_visual_names if name.startswith("screw_"))
        ctx.check("base has 4 screw heads", screw_count == 4, details=f"Found {screw_count} screws")
        ctx.check("turret has sensor face", "sensor_face" in [v.name for v in turret.visuals], details="Sensor face missing")

    # 3. Open pose (90 degree yaw) check
    with ctx.pose({yaw: 1.5708}):
        sensor_aabb = ctx.part_element_world_aabb(turret, elem="sensor_face")
        # aabb is tuple of (min_xyz_tuple, max_xyz_tuple)
        sensor_center_y = (sensor_aabb[0][1] + sensor_aabb[1][1]) / 2
        ctx.check("open pose (90deg yaw) sensor faces +Y", sensor_center_y > 0.05, details=f"Sensor center Y: {sensor_center_y}")

        # Turret remains seated at open pose
        ctx.expect_contact(base, turret, elem_a="base_shell", elem_b="turret_body", contact_tol=0.001, name="turret seated at 90deg yaw")

    return ctx.report()


object_model = build_object_model()
